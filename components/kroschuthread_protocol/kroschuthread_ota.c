#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "kroschuthread_ota.h"
#include "esp_partition.h"    
#include "esp_spiffs.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "kroschuthread_nodeid.h"
#include "kroschuthread_protocol.h"
#include "mbedtls/sha256.h"

#define TAG "KroschuOTA"
#define OTA_FILENAME "/spiffs/ota_fw.bin"
static FILE *ota_fp = NULL;

#include "freertos/queue.h"

#define OTA_QUEUE_SIZE 64
#define OTA_CHUNK_MAX 800

#define CHUNK_SIZE 768
#define MAX_CHUNKS 5000  // ak chceš väčšie súbory, kľudne zdvihni

typedef struct {
    uint8_t *data;
    size_t len;
} ota_chunk_t;

static QueueHandle_t ota_queue = NULL;
static TaskHandle_t ota_worker_task_handle = NULL;
static uint32_t g_chunk_size = CHUNK_SIZE;

static uint8_t g_expected_sha[32];
static bool    g_have_sha = false;

static void ota_writer_task(void *arg)
{
    ota_chunk_t item;
    for (;;) {
        if (xQueueReceive(ota_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (ota_fp) {
                fwrite(item.data, 1, item.len, ota_fp);
            }
        }
    }
}

void mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK)
        ESP_LOGI(TAG, "SPIFFS mounted: total=%u used=%u", (unsigned)total, (unsigned)used);
}


void print_ota_file_content() {
    FILE *f = fopen(OTA_FILENAME, "rb");
    if (!f) {
        printf("Nenašiel som OTA fw na %s!\n", OTA_FILENAME);
        return;
    }
    printf("Prvých 64 bytov /spiffs/ota_fw.bin:\n");
    uint8_t buf[64];
    int read = fread(buf, 1, 64, f);
    for (int i = 0; i < read; i++) {
        printf("%02X ", buf[i]);
        if ((i+1) % 16 == 0) printf("\n");
    }
    printf("\n");
    fclose(f);
}
void check_ota_file_size() {
    FILE *f = fopen(OTA_FILENAME, "rb");
    if (!f) { printf("n/a\n"); return; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    printf("Veľkosť ota_fw.bin: %ld bajtov\n", sz);
    fclose(f);
}

void delete_ota_file(void)
{
    int res = unlink(OTA_FILENAME);
    if (res == 0)
        ESP_LOGI(TAG, "Deleted old OTA file");
    else
        ESP_LOGW(TAG, "No OTA file to delete or unlink failed");
}

void kroschuthread_ota_flash_fw() {
    FILE *f = fopen(OTA_FILENAME, "rb");
    if (!f) {
        printf("Nenašiel som OTA fw na %s!\n", OTA_FILENAME);
        return;
    }

    // --- Vypočítaj SHA256 podpis ešte pred flashovaním ---
    uint8_t sig[32];
    uint8_t buf_sha[1024];
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);

    int n;
    while ((n = fread(buf_sha, 1, sizeof(buf_sha), f)) > 0)
        mbedtls_sha256_update(&ctx, buf_sha, n);

    mbedtls_sha256_finish(&ctx, sig);
    mbedtls_sha256_free(&ctx);

    // premeň na hex
    char sig_hex[65];
    for (int i = 0; i < 32; i++)
        sprintf(sig_hex + i * 2, "%02X", sig[i]);
    sig_hex[64] = 0;

    // zisti veľkosť
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    printf("[FLASH_READY] SHA256=%s  size=%ld\n", sig_hex, fsize);
    // --- koniec SHA256 výpisu ---

    // Toto je správne: zistí *ktorá* OTA partícia je voľná (nie factory)
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        printf("CHYBA: Nepodarilo sa nájsť OTA partíciu!\n");
        fclose(f);
        return;
    }

    printf("FLASHUJEM do partície %s @ 0x%" PRIx32 " veľkosť 0x%" PRIx32 "\n",
           update_partition->label, update_partition->address, update_partition->size);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        printf("CHYBA: esp_ota_begin zlyhalo! (%d)\n", err);
        fclose(f);
        return;
    }

    uint8_t buf[1024];
    int read;
    while ((read = fread(buf, 1, sizeof(buf), f)) > 0) {
        err = esp_ota_write(ota_handle, buf, read);
        if (err != ESP_OK) {
            printf("CHYBA: esp_ota_write zlyhalo! (%d)\n", err);
            esp_ota_end(ota_handle);
            fclose(f);
            return;
        }
    }
    fclose(f);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        printf("CHYBA: esp_ota_end zlyhalo! (%d)\n", err);
        return;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        printf("CHYBA: esp_ota_set_boot_partition zlyhalo! (%d)\n", err);
        return;
    }

    printf("OTA flash hotová, reštartujem!\n");
    esp_restart();
}



#define MAX_FRAGS_PER_MSG 64 

// kroschuthread_ota.c
#define REASM_SLOTS 6
typedef struct {
    bool used;
    uint16_t seq, total_frags, received_frags, src_node;
    uint64_t last_touch_ms;
    uint8_t* frag_data[MAX_FRAGS_PER_MSG];
    uint16_t frag_len [MAX_FRAGS_PER_MSG];
} reasm_slot_t;

static reasm_slot_t g_slots[REASM_SLOTS];

// kroschuthread_ota.c
static void reasm_slot_clear(reasm_slot_t* s){
    if (!s) return;
    for (int i = 0; i < MAX_FRAGS_PER_MSG; i++) {
        if (s->frag_data[i]) {
            free(s->frag_data[i]);
            s->frag_data[i] = NULL;
        }
        s->frag_len[i] = 0;
    }
    memset(s, 0, sizeof(*s));
}

static uint64_t now_ms(void){ return esp_timer_get_time()/1000ULL; }

static reasm_slot_t* reasm_pick(uint16_t seq, uint16_t src){
    // 1) existujúci
    for (int i=0;i<REASM_SLOTS;i++)
        if (g_slots[i].used && g_slots[i].seq==seq && g_slots[i].src_node==src)
            return &g_slots[i];
    // 2) free alebo najstarší
    int victim=-1; uint64_t oldest=UINT64_MAX;
    for (int i=0;i<REASM_SLOTS;i++){
        if (!g_slots[i].used){ victim=i; break; }
        if (g_slots[i].last_touch_ms < oldest){ oldest=g_slots[i].last_touch_ms; victim=i; }
    }
    if (victim>=0){ reasm_slot_clear(&g_slots[victim]); g_slots[victim].used=true;
        g_slots[victim].seq=seq; g_slots[victim].src_node=src; return &g_slots[victim]; }
    return NULL;
}

static reasm_slot_t g_reasm = {0};

static void reasm_reset(void) {
    for (int i = 0; i < MAX_FRAGS_PER_MSG; i++) {
        if (g_reasm.frag_data[i]) {
            free(g_reasm.frag_data[i]);
            g_reasm.frag_data[i] = NULL;
            g_reasm.frag_len[i]  = 0;
        }
    }
    memset(&g_reasm, 0, sizeof(g_reasm));
}


static uint32_t g_chunk_map[(MAX_CHUNKS+31)/32];
static uint32_t g_highest_chunk = 0;
static bool     g_end_seen = false;
static uint16_t g_ota_src_node = 0;      // kam posielať CNACK
static uint16_t g_last_msg_src_node = 0; // nastaví reasm pri každej komplet správe

// novinka:
static uint32_t g_expected_chunks = 0;   // spočítané z BEGIN (sz/cs), 0 = neznáme

static inline void cm_set(uint32_t i){ g_chunk_map[i>>5] |=  (1u<<(i&31)); }
static inline bool cm_get(uint32_t i){ return (g_chunk_map[i>>5]>>(i&31))&1; }
static inline void cm_clear_all(void){ memset(g_chunk_map, 0, sizeof(g_chunk_map)); }

static void send_cnack(uint16_t dest, uint32_t idx){
    if (!dest) return;
    uint16_t me = nodeid_get();
    uint32_t jitter_ms = 2 + ((me ^ idx) & 7); // 2..9ms
    vTaskDelay(pdMS_TO_TICKS(jitter_ms));
    char msg[32];
    int n = snprintf(msg, sizeof(msg), "CNACK|%u", (unsigned)idx);
    kroschuthread_protocol_send_data_no_ack((const uint8_t*)msg, n, dest, KROSCHUTHREAD_ACK_PORT);
}

void reasm_add_fragment(const uint8_t *data, size_t len,
                        uint16_t seq, uint16_t frag_idx, uint16_t total_frags,
                        uint16_t src_node)
{
    if (!data || len == 0) return;

    // --- Expirácia starých slotov (ľahký sweep pri každom volaní) ---
    {
        uint64_t now = now_ms();
        for (int i = 0; i < REASM_SLOTS; i++) {
            if (g_slots[i].used && (now - g_slots[i].last_touch_ms) > 1500 /*ms*/) {
                reasm_slot_clear(&g_slots[i]);
                memset(&g_slots[i], 0, sizeof(g_slots[i]));
            }
        }
    }

    // --- Vyber (alebo alokuj) slot pre (seq, src_node) ---
    reasm_slot_t *s = reasm_pick(seq, src_node);
    if (!s) {
        ESP_LOGW("Reasm", "No free reasm slot for seq=%u src=0x%04X", seq, src_node);
        return;
    }
    s->last_touch_ms = now_ms();
    s->src_node = src_node;

    // --- Inicializácia počtu fragmentov ---
    if (s->total_frags == 0) {
        if (total_frags == 0 || total_frags > MAX_FRAGS_PER_MSG) {
            ESP_LOGE("Reasm", "total_frags=%u out of range (max %u)", total_frags, MAX_FRAGS_PER_MSG);
            reasm_slot_clear(s);
            memset(s, 0, sizeof(*s));
            return;
        }
        s->seq          = seq;
        s->total_frags  = total_frags;
        s->received_frags = 0;
        // pole frag_data/frag_len by malo byť nulové po reasm_slot_clear()
    } else if (total_frags != 0 && total_frags != s->total_frags) {
        // Ak niektorý fragment nahlási väčšie total_frags (bežné pri oneskorenom príchode prvého),
        // zväčšíme len ak to dáva zmysel.
        if (total_frags > s->total_frags && total_frags <= MAX_FRAGS_PER_MSG) {
            s->total_frags = total_frags;
        } else if (total_frags < s->total_frags) {
            // menšie ignoruj – drž sa väčšieho (bezpečnejšie)
        } else {
            ESP_LOGW("Reasm", "Inconsistent total_frags=%u for seq=%u", total_frags, seq);
        }
    }

    // --- Kontroly indexu ---
    if (frag_idx >= s->total_frags) {
        ESP_LOGW("Reasm", "frag_idx %u out of bounds (total %u) for seq=%u",
                 frag_idx, s->total_frags, seq);
        return;
    }

    // --- Duplicitný fragment? ---
    if (s->frag_data[frag_idx] != NULL) {
        // dup – ignoruj
        return;
    }

    // --- Ulož fragment ---
    uint8_t *copy = (uint8_t *)malloc(len);
    if (!copy) {
        ESP_LOGE("Reasm", "malloc failed for frag %u len %u (seq=%u)",
                 frag_idx, (unsigned)len, seq);
        return;
    }
    memcpy(copy, data, len);
    s->frag_data[frag_idx] = copy;
    s->frag_len [frag_idx] = (uint16_t)len;
    s->received_frags++;
    s->last_touch_ms = now_ms();

    // --- Komplet? Poskladaj a odovzdaj ---
    if (s->received_frags == s->total_frags) {
        // spočítaj výslednú dĺžku
        size_t total_len = 0;
        for (uint16_t i = 0; i < s->total_frags; i++) {
            total_len += s->frag_len[i];
        }

        uint8_t *assembled = (uint8_t *)malloc(total_len);
        if (!assembled) {
            ESP_LOGE("Reasm", "malloc failed for assembled len %u (seq=%u)", (unsigned)total_len, seq);
            reasm_slot_clear(s);
            memset(s, 0, sizeof(*s));
            return;
        }

        // poskladaj v poradí 0..N-1
        size_t off = 0;
        for (uint16_t i = 0; i < s->total_frags; i++) {
            memcpy(assembled + off, s->frag_data[i], s->frag_len[i]);
            off += s->frag_len[i];
        }

        // sanity prefix – povoľ OTA_/OTA_END/CNACK|
        bool ok = false;
        if (total_len >= 4  && memcmp(assembled, "OTA_",    4) == 0) ok = true;
        if (total_len >= 7  && memcmp(assembled, "OTA_END", 7) == 0) ok = true;
        if (total_len >= 6  && memcmp(assembled, "CNACK|",  6) == 0) ok = true;

        if (!ok) {
            ESP_LOGW("Reasm", "Assembled doesn't start with OTA_/OTA_END/CNACK (len=%u) → drop", (unsigned)total_len);
            free(assembled);
            reasm_slot_clear(s);
            memset(s, 0, sizeof(*s));
            return;
        }

        // odovzdaj vyššej vrstve
        g_last_msg_src_node = src_node;
        ESP_LOGI("Reasm", "Full reassembled message, size=%u", (unsigned)total_len);
        kroschuthread_ota_process_message(assembled, total_len);

        free(assembled);
        reasm_slot_clear(s);
        memset(s, 0, sizeof(*s));
    }
}
// --- ACKy pre sendera ---
static void send_ackc(uint16_t dest, uint32_t idx) {
    if (!dest) return;
    char ack[24];
    int an = snprintf(ack, sizeof(ack), "ACKC|%u", (unsigned)idx);
    kroschuthread_protocol_send_data_no_ack((const uint8_t*)ack, an, dest, KROSCHUTHREAD_ACK_PORT);
}

static void send_ackr(uint16_t dest, uint32_t idx) {
    if (!dest) return;
    char ack[24];
    int an = snprintf(ack, sizeof(ack), "ACKR|%u", (unsigned)idx);
    kroschuthread_protocol_send_data_no_ack((const uint8_t*)ack, an, dest, KROSCHUTHREAD_ACK_PORT);
}
// --- CNACK sweeper (po END opakujeme žiadosti kým nie je komplet) ---
static TaskHandle_t g_cnack_sweeper_task = NULL;
static volatile bool g_cnack_sweeper_run = false;

static void cnack_sweeper_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(500); // každých 500 ms
    while (g_cnack_sweeper_run) {
        if (g_end_seen && g_ota_src_node) {
            bool complete = true;
            uint32_t top = g_expected_chunks ? g_expected_chunks : (g_highest_chunk + 1);
            // pošli len prvých pár chýbajúcich za tick (napr. 4 ks) nech nerobíme búrku
            int sent = 0;
            for (uint32_t i = 0; i < top; i++) {
                if (!cm_get(i)) {
                    complete = false;
                    send_cnack(g_ota_src_node, i);
                    if (++sent >= 4) break;
                }
            }
            if (complete) {
                // hotovo, vypneme seba
                g_cnack_sweeper_run = false;
                break;
            }
        }
        vTaskDelay(period);
    }
    g_cnack_sweeper_task = NULL;
    vTaskDelete(NULL);
}

// helper: parse 64-hex SHA -> 32 bytes
static bool parse_sha_hex64(const char *hex, uint8_t out[32]) {
    if (!hex) return false;
    for (int i = 0; i < 64; i++) {
        char c = hex[i];
        if (!((c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f'))) return false;
    }
    for (int i = 0; i < 32; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    return true;
}

// lokálny CRC16 rovnaký ako na senderi
static uint16_t crc16_ccitt_local(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// vypočítaj SHA súboru (na overenie)
static bool compute_file_sha256(const char *path, uint8_t out[32], long *fsize_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    uint8_t buf[1024];
    int n;
    long total = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        mbedtls_sha256_update(&ctx, buf, n);
        total += n;
    }
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    fclose(f);
    if (fsize_out) *fsize_out = total;
    return true;
}

static void ota_worker_task(void *arg)
{
    ota_chunk_t item;

    for (;;) {
        if (xQueueReceive(ota_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (item.len >= 4 && memcmp(item.data, "OTA_", 4) == 0) {

                // ---- OTA BEGIN ----
                if (memcmp(item.data, "OTA_BEGIN", 9) == 0) {
                    if (ota_fp) { fclose(ota_fp); ota_fp = NULL; }

                    cm_clear_all();
                    g_highest_chunk   = 0;
                    g_end_seen        = false;
                    g_ota_src_node    = g_last_msg_src_node;
                    g_expected_chunks = 0;
                    g_have_sha        = false;

                    const char *p  = (const char*)item.data;
                    const char *sz = strstr(p, "|sz=");
                    const char *cs = strstr(p, "|cs=");
                    const char *sh = strstr(p, "|sha=");
                    uint32_t file_sz = 0;
                    if (sz) file_sz = (uint32_t)strtoul(sz + 4, NULL, 10);

                    g_chunk_size = CHUNK_SIZE;
                    if (cs) {
                        uint32_t cs_val = (uint32_t)strtoul(cs + 4, NULL, 10);
                        if (cs_val > 0 && cs_val <= 1024) g_chunk_size = cs_val;
                    }
                    if (file_sz > 0) {
                        g_expected_chunks = (file_sz + g_chunk_size - 1) / g_chunk_size;
                        if (g_expected_chunks > MAX_CHUNKS) g_expected_chunks = MAX_CHUNKS;
                    }

                    if (sh && strlen(sh + 5) >= 64) {
                        char tmp[65]; memcpy(tmp, sh + 5, 64); tmp[64] = 0;
                        g_have_sha = parse_sha_hex64(tmp, g_expected_sha);
                    }

                    ota_fp = fopen(OTA_FILENAME, "wb+");
                    if (ota_fp) {
                        ESP_LOGI(TAG, "OTA BEGIN (src=0x%04X, expected_chunks=%u, cs=%u, sha=%s)",
                                 g_ota_src_node, (unsigned)g_expected_chunks, (unsigned)g_chunk_size,
                                 g_have_sha ? "yes" : "no");
                    } else {
                        ESP_LOGE(TAG, "Failed to open %s for write", OTA_FILENAME);
                    }

                    // vypni starý sweeper ak beží
                    g_cnack_sweeper_run = false;
                    if (g_cnack_sweeper_task) { vTaskDelay(1); }
                }

                // ---- OTA CHUNK (s voliteľným c=XXXX) ----
                else if (memcmp(item.data, "OTA_CHUNK", 9) == 0) {
                    const char *first_sep  = strchr((const char *)item.data, '|');           // po "OTA_CHUNK"
                    const char *second_sep = first_sep ? strchr(first_sep + 1, '|') : NULL; // po indexe
                    if (!first_sep || !second_sep) goto cleanup;

                    char idx_buf[12] = {0};
                    size_t idx_len = (size_t)(second_sep - (first_sep + 1));
                    if (idx_len >= sizeof(idx_buf)) idx_len = sizeof(idx_buf)-1;
                    memcpy(idx_buf, first_sep + 1, idx_len);
                    uint32_t chunk_idx = (uint32_t)strtoul(idx_buf, NULL, 10);

                    const uint8_t *cursor = (const uint8_t *)(second_sep + 1);
                    const uint8_t *chunk_data = cursor;
                    size_t chunk_size = 0;

                    // voliteľný "c=XXXX|"
                    bool have_crc = false;
                    uint16_t exp_crc = 0;
                    if (cursor[0] == 'c' && cursor[1] == '=') {
                        // načítaj 4 hex (bez validácie -> safe enough)
                        char hx[5] = {0};
                        for (int i = 0; i < 4 && cursor[2+i]; i++) hx[i] = (char)cursor[2+i];
                        exp_crc = (uint16_t)strtoul(hx, NULL, 16);
                        const char *third = strchr((const char*)cursor, '|');
                        if (!third) goto cleanup;
                        chunk_data = (const uint8_t *)(third + 1);
                        have_crc = true;
                    }

                    chunk_size = item.len - (chunk_data - item.data);
                    if (!g_ota_src_node) g_ota_src_node = g_last_msg_src_node;

                    if (ota_fp && chunk_size > 0) {
                        // CRC check, ak je k dispozícii
                        if (have_crc) {
                            uint16_t got = crc16_ccitt_local(chunk_data, chunk_size);
                            if (got != exp_crc) {
                                ESP_LOGW(TAG, "CRC mismatch for idx %u (got %04X, exp %04X) → CNACK",
                                         (unsigned)chunk_idx, got, exp_crc);
                                send_cnack(g_ota_src_node, chunk_idx);
                                goto cleanup; // neukladaj korupciu
                            }
                        }

                        if (chunk_idx >= MAX_CHUNKS) goto cleanup;
                        bool was_missing = !cm_get(chunk_idx);

                        fseek(ota_fp, (long)chunk_idx * (long)g_chunk_size, SEEK_SET);
                        fwrite(chunk_data, 1, chunk_size, ota_fp);
                        cm_set(chunk_idx);
                        if (chunk_idx > g_highest_chunk) g_highest_chunk = chunk_idx;

                        // ACK za chunk (vždy)
                        send_ackc(g_ota_src_node, chunk_idx);
                        // ak je to repair po END → ACKR
                        if (was_missing && g_end_seen) send_ackr(g_ota_src_node, chunk_idx);

                        if (g_expected_chunks && chunk_idx >= g_expected_chunks) {
                            ESP_LOGW(TAG, "Chunk idx %u >= expected %u (ignoring extra)",
                                     (unsigned)chunk_idx, (unsigned)g_expected_chunks);
                        }

                        // ak už prišiel END, skús komplet a SHA
                        if (g_end_seen) {
                            bool complete = true;
                            uint32_t top = g_expected_chunks ? g_expected_chunks : (g_highest_chunk + 1);
                            for (uint32_t i = 0; i < top; i++) {
                                if (!cm_get(i)) { complete = false; break; }
                            }
                            if (complete) {
                                if (g_have_sha) {
                                    uint8_t sha_now[32];
                                    long fsz = 0;
                                    fflush(ota_fp);
                                    // neuzatvárame, ale uistíme sa, že je sync
                                    if (compute_file_sha256(OTA_FILENAME, sha_now, &fsz)) {
                                        if (memcmp(sha_now, g_expected_sha, 32) != 0) {
                                            ESP_LOGE(TAG, "SHA mismatch after END (size=%ld) → full re-request", fsz);
                                            // vyžiadaj celé znova (reset bitmapy) a spusti sweeper
                                            cm_clear_all();
                                            // udrž aktuálny g_highest_chunk kvôli top
                                            if (g_expected_chunks) g_highest_chunk = g_expected_chunks - 1;
                                            if (!g_cnack_sweeper_task) {
                                                g_cnack_sweeper_run = true;
                                                xTaskCreatePinnedToCore(cnack_sweeper_task, "cnack_sweeper",
                                                                        4096, NULL, 4, &g_cnack_sweeper_task, 0);
                                            }
                                        } else {
                                            if (ota_fp) { fclose(ota_fp); ota_fp = NULL; }
                                            ESP_LOGI(TAG, "All chunks OK + SHA match → flashing");
                                            g_cnack_sweeper_run = false;
                                            kroschuthread_ota_flash_fw();
                                        }
                                    }
                                } else {
                                    if (ota_fp) { fclose(ota_fp); ota_fp = NULL; }
                                    ESP_LOGI(TAG, "All chunks present after END → flashing");
                                    g_cnack_sweeper_run = false;
                                    kroschuthread_ota_flash_fw();
                                }
                            }
                        }
                    }
                }

                // ---- OTA END ----
                else if (memcmp(item.data, "OTA_END", 7) == 0) {
                    g_end_seen = true;
                    bool complete = true;
                    uint32_t top = g_expected_chunks ? g_expected_chunks : (g_highest_chunk + 1);

                    int printed = 0;
                    for (uint32_t i = 0; i < top; i++) {
                        if (!cm_get(i)) {
                            complete = false;
                            if (printed < 10) {
                                ESP_LOGW(TAG, "Missing chunk idx=%u", (unsigned)i);
                                printed++;
                            }
                        }
                    }

                    if (!complete) {
                        ESP_LOGW(TAG, "OTA_END prišlo, chýbajú chunky – spúšťam CNACK sweeper");
                        if (!g_cnack_sweeper_task) {
                            g_cnack_sweeper_run = true;
                            xTaskCreatePinnedToCore(cnack_sweeper_task, "cnack_sweeper",
                                                    4096, NULL, 4, &g_cnack_sweeper_task, 0);
                        }
                    } else {
                        // komplet – over SHA (ak máme), inak rovno flash
                        if (g_have_sha) {
                            uint8_t sha_now[32];
                            long fsz = 0;
                            if (compute_file_sha256(OTA_FILENAME, sha_now, &fsz) &&
                                memcmp(sha_now, g_expected_sha, 32) == 0) {
                                if (ota_fp) { fclose(ota_fp); ota_fp = NULL; }
                                ESP_LOGI(TAG, "OTA END → SHA OK (size=%ld), flashujem...", fsz);
                                kroschuthread_ota_flash_fw();
                            } else {
                                ESP_LOGE(TAG, "OTA END → SHA mismatch, vyžiadam celé znova");
                                cm_clear_all();
                                if (g_expected_chunks) g_highest_chunk = g_expected_chunks - 1;
                                if (!g_cnack_sweeper_task) {
                                    g_cnack_sweeper_run = true;
                                    xTaskCreatePinnedToCore(cnack_sweeper_task, "cnack_sweeper",
                                                            4096, NULL, 4, &g_cnack_sweeper_task, 0);
                                }
                            }
                        } else {
                            if (ota_fp) { fclose(ota_fp); ota_fp = NULL; }
                            ESP_LOGI(TAG, "OTA END → všetko prišlo, flashujem...");
                            kroschuthread_ota_flash_fw();
                        }
                    }
                }
            }

        cleanup:
            free(item.data);
        }
    }
}

// ====== INIT QUEUE & TASK ======
void ota_queue_init(void)
{
    if (!ota_queue)
        ota_queue = xQueueCreate(256, sizeof(ota_chunk_t)); // 🔹 až 64 chunkov vo fronte

    if (!ota_worker_task_handle)
        xTaskCreatePinnedToCore(ota_worker_task, "ota_worker_task", 8192, NULL, 5, &ota_worker_task_handle, 0);
}

// ====== MESSAGE ENTRY POINT ======
void kroschuthread_ota_process_message(const uint8_t *payload, size_t len)
{
    if (!ota_queue)
        ota_queue_init();

    // 🔹 Alokuj kopiu dát, aby fronta mohla bezpečne uchovávať chunk
    ota_chunk_t item;
    item.data = malloc(len);
    if (!item.data) {
        ESP_LOGE(TAG, "Malloc failed for OTA chunk");
        return;
    }
    memcpy(item.data, payload, len);
    item.len = len;

    // 🔹 Pokus o vloženie do fronty
    if (xQueueSend(ota_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "OTA queue full — dropping chunk (len=%u)", (unsigned)len);
        free(item.data);
    }
}
