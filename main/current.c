#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>

#include "mbedtls/sha256.h"

#include "kroschuthread_protocol.h"
#include "kroschuthread_nodeid.h"
#include "kroschuthread_ota.h"   // mount_spiffs(), delete_ota_file(), kroschuthread_ota_process_message()

// ====================== CONFIG / DEMO MODE ======================
static const char* TAG = "KroschuThreadMain";

#define DEMO_MODE_SENDER     1
#define DEMO_MODE_RECEIVER   2
#define DEMO_MODE DEMO_MODE_SENDER  // prepni na RECEIVER, keď chceš prijímač

// ====================== OTA PARAMS ==============================
#define CHUNK_SIZE            768     // musí sedieť s cs= v BEGIN
#define MAX_CHUNKS            5000

// stream ACK (ACKC) – koľko čakáme na ACK jedného chunku
#define ACK_TIMEOUT_MS        150
#define MAX_ACK_RETRIES       5

// repair worker (ACKR) – po END dopĺňanie chýbajúcich
#define REPAIR_ACK_TIMEOUT_MS 250
#define REPAIR_MAX_TRIES      8
#define REPAIR_RTO_MS         300     // min. rozostup opakovaného poslania rovnakého idx

static const char *SENDER_OTA_PATH = "/spiffs/pexis.bin";

// ====================== SENDER STATE ============================
static uint32_t g_total_chunks = 0;
static uint32_t g_file_size    = 0;
static volatile uint32_t g_stream_idx = UINT32_MAX;


static uint32_t g_ack_bitmap[(MAX_CHUNKS + 31) / 32];     // potvrdené idx (ACKC/ACKR)
static uint32_t g_pending_bitmap[(MAX_CHUNKS + 31) / 32]; // naposledy poslané (anti-spam)
static uint8_t  g_retry_cnt[MAX_CHUNKS];                  // repair retry counter
static uint32_t g_last_tx_ms[MAX_CHUNKS];                 // posledný TX čas (ms)

// fronty pre notifikácie
static QueueHandle_t s_ack_queue   = NULL; // ACKC|idx
static QueueHandle_t s_ackr_queue  = NULL; // ACKR|idx
static QueueHandle_t s_cnack_queue = NULL; // CNACK požiadavky (idx)

static volatile bool s_ota_streaming = false; // počas primárneho streamu ignoruj CNACK

// ====================== UTILS ==============================
static inline uint32_t now_ms_sender(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }
static inline void bm_set(uint32_t *bm, uint32_t i){ bm[i>>5] |=  (1u<<(i&31)); }
static inline void bm_clr(uint32_t *bm, uint32_t i){ bm[i>>5] &= ~(1u<<(i&31)); }
static inline bool bm_get(uint32_t *bm, uint32_t i){ return (bm[i>>5]>>(i&31)) & 1u; }

// CRC16-CCITT (poly 0x1021), init 0xFFFF – rovnaké na oboch stranách
static uint16_t crc16_ccitt_local(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// SHA súboru (pre BEGIN sha=)
static void compute_sha256(const char *path, uint8_t out[32], uint32_t *fsize)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "[SHA] Failed to open %s", path);
        memset(out, 0, 32);
        if (fsize) *fsize = 0;
        return;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    if (fsize) *fsize = (uint32_t)size;
    fseek(f, 0, SEEK_SET);

    mbedtls_sha256_context ctx; mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    uint8_t buf[1024];
    int n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        mbedtls_sha256_update(&ctx, buf, n);
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    fclose(f);
}

// Pošli REPAIR chunk (s CRC) – samotné poslanie (bez čakania na ACKR)
static void send_repair_chunk(uint32_t idx)
{
    if (idx >= g_total_chunks) return;

    // anti-spam / backoff
    uint32_t now = now_ms_sender();
    if (bm_get(g_pending_bitmap, idx)) {
        if (now - g_last_tx_ms[idx] < REPAIR_RTO_MS) return;
    }

    FILE *f = fopen(SENDER_OTA_PATH, "rb");
    if (!f) return;
    if (fseek(f, (long)idx * (long)CHUNK_SIZE, SEEK_SET) != 0) { fclose(f); return; }

    size_t want = CHUNK_SIZE;
    if ((idx + 1) * CHUNK_SIZE > g_file_size) {
        uint32_t remain = g_file_size - idx * CHUNK_SIZE;
        want = remain;
    }

    uint8_t chunk[CHUNK_SIZE];
    size_t nread = fread(chunk, 1, want, f);
    fclose(f);
    if (nread == 0) return;

    uint16_t csum = crc16_ccitt_local(chunk, nread);

    char header[48];
    int  hn = snprintf(header, sizeof(header), "OTA_CHUNK|%u|c=%04X|", (unsigned)idx, (unsigned)csum);

    uint8_t *buf = (uint8_t*)malloc(hn + nread);
    if (!buf) return;
    memcpy(buf, header, hn);
    memcpy(buf + hn, chunk, nread);

    kroschuthread_protocol_send_data_no_ack(buf, hn + nread, 0xFFFF, 1234);
    free(buf);

    bm_set(g_pending_bitmap, idx);
    g_last_tx_ms[idx] = now;
    if (g_retry_cnt[idx] < 255) g_retry_cnt[idx]++;

    vTaskDelay(pdMS_TO_TICKS(5 + (idx % 7))); // jemný jitter
}

// ================ DEBUG / PROTOCOL CALLBACKS ====================
void mesh_debug_task(void) {
    node_table_debug_print();
}

void ack_received_callback(uint16_t acked_sequence)
{
    ESP_LOGI(TAG, "LL-ACK received for sequence %u", (unsigned)acked_sequence);
}
// SENDER: príjem jednorazových správ (ACKC/ACKR/CNACK). OTA_* na SENDERI ignorujeme.
void data_received_callback(const uint8_t* data, size_t data_length, uint16_t source_port)
{
    (void)source_port;

    // 1) OTA_* správy sú pre RX, na SENDERI ich neriešime
    if (data_length >= 4 && memcmp(data, "OTA_", 4) == 0) {
        return;
    }

    // 2) ACKC|<idx> – chunk potvrdený pri primárnom streame
    if (data_length >= 5 && memcmp(data, "ACKC|", 5) == 0) {
        uint32_t idx = 0;
        for (size_t i = 5; i < data_length; i++) {
            if (data[i] < '0' || data[i] > '9') break;
            idx = idx * 10 + (uint32_t)(data[i] - '0');
        }
        if (idx < MAX_CHUNKS) {
            bm_set(g_ack_bitmap, idx);
            bm_clr(g_pending_bitmap, idx);
            if (s_ack_queue) xQueueSend(s_ack_queue, &idx, 0);
        }
        return;
    }

    // 3) ACKR|<idx> – potvrdenie opravného chunku po END
    if (data_length >= 5 && memcmp(data, "ACKR|", 5) == 0) {
        uint32_t idx = 0;
        for (size_t i = 5; i < data_length; i++) {
            if (data[i] < '0' || data[i] > '9') break;
            idx = idx * 10 + (uint32_t)(data[i] - '0');
        }
        if (idx < MAX_CHUNKS) {
            bm_set(g_ack_bitmap, idx);
            bm_clr(g_pending_bitmap, idx);
            if (s_ackr_queue) xQueueSend(s_ackr_queue, &idx, 0);
        }
        return;
    }

    // 4) CNACK|<idx> – vyžiadanie doplnenia chunku (už NEignorujeme počas streamu)
    if (data_length >= 6 && memcmp(data, "CNACK|", 6) == 0) {
        uint32_t idx = 0;
        for (size_t i = 6; i < data_length; i++) {
            if (data[i] < '0' || data[i] > '9') break;
            idx = idx * 10 + (uint32_t)(data[i] - '0');
        }
        if (idx >= g_total_chunks) return;
        if (bm_get(g_ack_bitmap, idx)) return;               // už potvrdené? nič
        if (g_retry_cnt[idx] >= REPAIR_MAX_TRIES) return;    // už sme to skúšali dosť

        // Ak je to práve posielaný chunk, pošli okamžite repair (s anti-spam backoffom vo vnútri)
        if (idx == g_stream_idx) {
            send_repair_chunk(idx);
            return;
        }

        // Inak to pošli do opravárskej fronty – worker si to vybaví (aj počas streamu)
        if (s_cnack_queue) xQueueSend(s_cnack_queue, &idx, 0);
        return;
    }

    // 5) Iné payloady – voliteľne debug
}

static void repair_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t idx = 0;
        if (xQueueReceive(s_cnack_queue, &idx, portMAX_DELAY) != pdTRUE) continue;

        int tries = 0;
        for (;;) {
            // počas streamu opravuj len tie indexy, ktoré sú <= práve posielaný
            while (s_ota_streaming && idx > g_stream_idx) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            send_repair_chunk(idx); // pošle s CRC, nastaví pending/backoff

            // čakaj na ACKR|idx (alebo ACKC, ak RX ešte nevidel END – povolíme oboje)
            bool ok = false;
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(REPAIR_ACK_TIMEOUT_MS);
            while (xTaskGetTickCount() < deadline) {
                uint32_t aidx = UINT32_MAX;
                if (s_ackr_queue && xQueueReceive(s_ackr_queue, &aidx, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (aidx == idx) { ok = true; break; }
                }
                if (s_ack_queue && xQueueReceive(s_ack_queue, &aidx, 0) == pdTRUE) {
                    if (aidx == idx) { ok = true; break; }
                }
            }
            if (ok) {
                ESP_LOGI("SENDER", "Repair OK idx %u", (unsigned)idx);
                break;
            }
            if (++tries > REPAIR_MAX_TRIES) {
                ESP_LOGE("SENDER", "Repair FAIL idx %u (no ACKR/ACKC)", (unsigned)idx);
                break;
            }
            ESP_LOGW("SENDER", "Repair retry idx %u", (unsigned)idx);
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
}

void sender_task(void* parameters)
{
    ESP_LOGI(TAG, "Starting SENDER (OTA sender)…");

    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = 20,
        .data_callback = data_received_callback,
        .ack_callback = ack_received_callback
    };

    if (kroschuthread_protocol_init(&config) != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize protocol");
        vTaskDelete(NULL);
        return;
    }

    // fronty
    if (!s_ack_queue)   s_ack_queue   = xQueueCreate(64,  sizeof(uint32_t));
    if (!s_ackr_queue)  s_ackr_queue  = xQueueCreate(64,  sizeof(uint32_t));
    if (!s_cnack_queue) s_cnack_queue = xQueueCreate(128, sizeof(uint32_t));

    // repair worker
    static TaskHandle_t s_repair_task = NULL;
    if (!s_repair_task) {
        xTaskCreatePinnedToCore(repair_worker_task, "repair_worker", 4096, NULL, 5, &s_repair_task, 0);
    }

    // súbor
    FILE *f = fopen(SENDER_OTA_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open OTA file: %s", SENDER_OTA_PATH);
        vTaskDelete(NULL);
        return;
    }

    // veľkosť + SHA
    uint8_t sha[32]; uint32_t fsz = 0;
    compute_sha256(SENDER_OTA_PATH, sha, &fsz);
    g_file_size = fsz;
    g_total_chunks = (g_file_size + CHUNK_SIZE - 1) / CHUNK_SIZE;

    char sha_hex[65];
    for (int i = 0; i < 32; i++) sprintf(&sha_hex[i*2], "%02X", sha[i]);
    sha_hex[64] = 0;

    // BEGIN
    s_ota_streaming = true;
    g_stream_idx = UINT32_MAX;
    char begin_msg[160];
    snprintf(begin_msg, sizeof(begin_msg), "OTA_BEGIN|pexis.bin|sz=%lu|cs=%u|sha=%s",
             (unsigned long)g_file_size, CHUNK_SIZE, sha_hex);
    kroschuthread_protocol_send_data_no_ack((const uint8_t *)begin_msg, strlen(begin_msg), 0xFFFF, 1234);
    vTaskDelay(pdMS_TO_TICKS(300));

    // stream s per-chunk ACK (ACKC)
    uint8_t chunk[CHUNK_SIZE];
    uint32_t idx = 0;
    uint32_t total_sent = 0;

    while (idx < g_total_chunks) {
        size_t nread = fread(chunk, 1, CHUNK_SIZE, f);
        if (nread == 0) break;

        uint16_t csum = crc16_ccitt_local(chunk, nread);
        char header[48];
        int  hn = snprintf(header, sizeof(header), "OTA_CHUNK|%u|c=%04X|", (unsigned)idx, (unsigned)csum);

        uint8_t *buf = (uint8_t*)malloc(hn + nread);
        if (!buf) { ESP_LOGE(TAG, "malloc failed"); break; }
        memcpy(buf, header, hn);
        memcpy(buf + hn, chunk, nread);

        int retries = 0;
        for (;;) {
            g_stream_idx = idx;  // 👈 dôležité: označ „práve posielaný“ chunk

            kroschuthread_status_t st =
                kroschuthread_protocol_send_data_no_ack(buf, hn + nread, 0xFFFF, 1234);
            if (st != KROSCHUTHREAD_STATUS_SUCCESS) {
                ESP_LOGW(TAG, "Send fail for chunk %u (st=%d) → retry", (unsigned)idx, st);
                if (++retries > MAX_ACK_RETRIES) { free(buf); goto stream_end; }
                vTaskDelay(pdMS_TO_TICKS(15));
                continue;
            }

            // čakaj ACKC|idx (repary na ten istý idx môžu prichádzať aj cez ACKR, uznávame obe)
            bool ok = false;
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
            while (xTaskGetTickCount() < deadline) {
                uint32_t aidx = UINT32_MAX;
                if (xQueueReceive(s_ack_queue, &aidx, pdMS_TO_TICKS(10)) == pdTRUE && aidx == idx) { ok = true; break; }
                if (xQueueReceive(s_ackr_queue, &aidx, 0) == pdTRUE && aidx == idx) { ok = true; break; }
            }
            if (!ok) {
                if (++retries > MAX_ACK_RETRIES) {
                    ESP_LOGE(TAG, "No ACKC/ACKR for chunk %u, giving up", (unsigned)idx);
                    free(buf);
                    goto stream_end;
                }
                ESP_LOGW(TAG, "Retry chunk %u (ACK timeout)", (unsigned)idx);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            break; // ACK prišiel
        }

        total_sent += nread;
        ESP_LOGI(TAG, "Sent chunk #%u OK (%u bytes, total %u)",
                 (unsigned)idx, (unsigned)nread, (unsigned)total_sent);

        free(buf);
        idx++;
        vTaskDelay(pdMS_TO_TICKS(10)); // pacing
    }

stream_end:
    fclose(f);
    kroschuthread_protocol_send_data_no_ack((const uint8_t *)"OTA_END", 7, 0xFFFF, 1234);
    s_ota_streaming = false;
    g_stream_idx = UINT32_MAX;

    ESP_LOGI(TAG, "Sender done, size=%u, chunks=%u",
             (unsigned)g_file_size, (unsigned)g_total_chunks);
    vTaskDelete(NULL);
}


// RX wrapper: prispôsobí podpis callbacku (pridá tretí argument, ktorý ignorujeme)
static void receiver_data_cb(const uint8_t *data, size_t len, uint16_t src_port)
{
    (void)src_port; // nepoužívame, aby nehučal warning
    kroschuthread_ota_process_message(data, len);
}


// ===================== RECEIVER TASK (len demo smyčka) ==========
void receiver_task(void* parameters)
{
    ESP_LOGI(TAG, "Starting RECEIVER demo…");

    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = KROSCHUTHREAD_TX_POWER,
        .data_callback = receiver_data_cb, // RX posiela všetko do OTA pipeline
        .ack_callback = ack_received_callback
    };

    if (kroschuthread_protocol_init(&config) != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize protocol");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Protocol ready – waiting for OTA…");
    uint32_t stats_counter = 0;

    for (;;) {
        if (++stats_counter % 30 == 0) {
            kroschuthread_stats_t stats;
            if (kroschuthread_protocol_get_stats(&stats) == KROSCHUTHREAD_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Stats TX=%" PRIu32 " RX=%" PRIu32 " ACKs sent=%" PRIu32
                             " ACKs recv=%" PRIu32 " CRCerr=%" PRIu32,
                         stats.frames_transmitted, stats.frames_received,
                         stats.acks_sent, stats.acks_received, stats.crc_errors);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===================== APP MAIN =================================
void app_main(void)
{
    mount_spiffs();      // z kroschuthread_ota.c
    delete_ota_file();   // vymaz starý OTA súbor

    ESP_LOGI(TAG, "KroschuThread Protocol Demo Starting…");
    mesh_debug_task();

    // NVS (PHY kalibrácia)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS OK");

    ESP_LOGI(TAG, "ESP32-C6 802.15.4 Custom Protocol");
    ESP_LOGI(TAG, "Channel=%d  DataPort=%d  AckPort=%d  MaxPayload=%d",
             KROSCHUTHREAD_CHANNEL, KROSCHUTHREAD_DATA_PORT,
             KROSCHUTHREAD_ACK_PORT, KROSCHUTHREAD_MAX_PAYLOAD_SIZE);

#if DEMO_MODE == DEMO_MODE_SENDER
    ESP_LOGI(TAG, "Running in SENDER mode");
    xTaskCreate(sender_task, "sender_task", 8192, NULL, 5, NULL);
#elif DEMO_MODE == DEMO_MODE_RECEIVER
    ESP_LOGI(TAG, "Running in RECEIVER mode");
    xTaskCreate(receiver_task, "receiver_task", 4096, NULL, 5, NULL);
#else
    #error "Invalid DEMO_MODE configured!"
#endif
}
