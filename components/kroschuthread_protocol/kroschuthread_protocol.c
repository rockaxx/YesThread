#include "kroschuthread_protocol.h"
#include "kroschuthread_radio.h"
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "esp_ieee802154.h"

static const char* TAG = "KroschuThread";


// ==== Internal state ====
typedef struct {
    bool initialized;
    uint16_t next_sequence_number;
    kroschuthread_config_t config;
    kroschuthread_stats_t stats;
    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
    SemaphoreHandle_t stats_mutex;
    TaskHandle_t protocol_task_handle;
} kroschuthread_protocol_state_t;

typedef struct {
    uint8_t frame_data[KROSCHUTHREAD_MAX_FRAME_SIZE];
    size_t frame_length;
    int64_t timestamp_us;
} kroschuthread_frame_queue_item_t;

static kroschuthread_protocol_state_t g_protocol_state = {0};

// ==== TX pipeline primitives ====
// V ISR už nerobíme TX – len budíme TX task
static TaskHandle_t s_tx_task_handle = NULL;

// Volá sa z ISR (radio tx_done) – len notify, nič veľké
static void IRAM_ATTR proto_on_tx_ready(void)
{
    BaseType_t woke = pdFALSE;
    if (s_tx_task_handle) {
        vTaskNotifyGiveFromISR(s_tx_task_handle, &woke);
        if (woke) portYIELD_FROM_ISR();
    }
}

#if KROSCHUTHREAD_RECEIVER_ENABLED
// RX shim volaná z ISR (cez radio RX callback)
typedef struct {
    uint8_t len;
    uint8_t buf[256]; // 802.15.4 payload (bez FCS), real world do ~120
} proto_rx_item_t;

static void IRAM_ATTR proto_rx_isr_shim(const uint8_t *payload, size_t len)
{
    if (!payload || len == 0 || len > sizeof(((proto_rx_item_t*)0)->buf)) return;

    proto_rx_item_t item;
    item.len = (uint8_t)len;
    // POZOR: v ISR si to MUSÍŠ skopírovať, buffer od drivera nebude žiť po receive_handle_done
    memcpy(item.buf, payload, item.len);

    BaseType_t woke = pdFALSE;
    if (g_protocol_state.rx_queue) {
        xQueueSendFromISR(g_protocol_state.rx_queue, &item, &woke);
        if (woke) portYIELD_FROM_ISR();
    }
}

// Worker task – už mimo ISR, tu môžeš logovať a brať mutexy
static void rx_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        proto_rx_item_t item;
        if (xQueueReceive(g_protocol_state.rx_queue, &item, portMAX_DELAY) == pdTRUE) {

            // bezpečne uprav štatistiky
            if (g_protocol_state.stats_mutex &&
                xSemaphoreTake(g_protocol_state.stats_mutex, portMAX_DELAY) == pdTRUE) {
                g_protocol_state.stats.frames_received++;
                xSemaphoreGive(g_protocol_state.stats_mutex);
            }

            // zavolaj užívateľský callback
            if (g_protocol_state.config.data_callback) {
                g_protocol_state.config.data_callback(item.buf, item.len, KROSCHUTHREAD_DATA_PORT);
            }
        }
    }
}
#endif // KROSCHUTHREAD_RECEIVER_ENABLED

// Vysokoprioritný TX task, ktorý drainuje frontu bez tick-jitteru
static void tx_pipeline_task(void* arg)
{
    (void)arg;
    // Čerstvý pokus po boote – ak je niečo vo fronte, pošli hneď
    ulTaskNotifyTake(pdTRUE, 0);

    for (;;) {
        // Počkám, kým príde notify z ISR alebo od send path
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Drain frontu: vždy pošlem presne 1 rámec, zvyšok nechá ďalšiemu notify (kvôli férovosti a stabilnému IPG)
        kroschuthread_frame_queue_item_t item;
        while (uxQueueMessagesWaiting(g_protocol_state.tx_queue) > 0) {
            if (xQueueReceive(g_protocol_state.tx_queue, &item, 0) != pdTRUE)
                break;

            kroschuthread_status_t st = kroschuthread_radio_transmit(item.frame_data, item.frame_length);
            if (st == KROSCHUTHREAD_STATUS_SUCCESS) {
                // Počkáme na tx_done → ISR nás znova zobudí notify
                break;
            }
            // Ak je BUFFER_FULL, vráť späť a zavolaj yield (nemalo by sa stať, ale buďme mäkkí)
            if (st == KROSCHUTHREAD_STATUS_BUFFER_FULL) {
                xQueueSendToFront(g_protocol_state.tx_queue, &item, 0);
                // Malý mikroyield, necháme rádio dýchnuť
                taskYIELD();
                break;
            }
            // Ak ERROR, skús ďalší item
        }
    }
}

// ===========================================================
// INIT – nastav callback na ISR pipeline
// ===========================================================
kroschuthread_status_t kroschuthread_protocol_init(const kroschuthread_config_t* config)
{
    if (!config) return KROSCHUTHREAD_STATUS_ERROR;
    if (g_protocol_state.initialized) return KROSCHUTHREAD_STATUS_SUCCESS;
    ESP_LOGI(TAG, "Initializing KroschuThread protocol (RX+TX pipeline)");

    memcpy(&g_protocol_state.config, config, sizeof(kroschuthread_config_t));
    g_protocol_state.next_sequence_number = 1;
    memset(&g_protocol_state.stats, 0, sizeof(kroschuthread_stats_t));

    // FRONTY – len to, čo reálne používame
    g_protocol_state.tx_queue = xQueueCreate(64, sizeof(kroschuthread_frame_queue_item_t));
#if KROSCHUTHREAD_RECEIVER_ENABLED
    g_protocol_state.rx_queue = xQueueCreate(32, sizeof(proto_rx_item_t));
#endif
    g_protocol_state.stats_mutex = xSemaphoreCreateMutex();

    // Over alokácie – ak RX vypnuté, rx_queue sa nekontroluje
    if (!g_protocol_state.tx_queue || !g_protocol_state.stats_mutex
#if KROSCHUTHREAD_RECEIVER_ENABLED
        || !g_protocol_state.rx_queue
#endif
    ) {
        ESP_LOGE(TAG, "Queue/mutex alloc failed (rx=%p tx=%p mtx=%p)",
                 g_protocol_state.rx_queue, g_protocol_state.tx_queue, g_protocol_state.stats_mutex);
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    // Rádio dostane ISR-safe RX shim iba ak je povolený RX
    kroschuthread_radio_config_t radio_config = {
        .channel    = config->channel,
        .tx_power   = config->tx_power,
#if KROSCHUTHREAD_RECEIVER_ENABLED
        .rx_callback = (config->data_callback ? proto_rx_isr_shim : NULL),
#else
        .rx_callback = NULL,
#endif
    };

    if (kroschuthread_radio_init(&radio_config) != KROSCHUTHREAD_STATUS_SUCCESS)
        return KROSCHUTHREAD_STATUS_ERROR;

    // Zaregistruj TX-done notifikáciu → odblokuje ďalšie snímky v pipeline
    kroschuthread_radio_set_next_tx_callback(proto_on_tx_ready);

#if KROSCHUTHREAD_RECEIVER_ENABLED
    if (radio_config.rx_callback) {
        kroschuthread_radio_set_promiscuous(true);
        kroschuthread_radio_start_rx();
        // RX worker – spracuje frontu mimo ISR (bez pádov)
        if (xTaskCreate(rx_worker_task, "rx_worker", 8192, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create RX worker task");
            return KROSCHUTHREAD_STATUS_ERROR;
        }
    }
#endif

    // TX pipeline task
    if (xTaskCreate(tx_pipeline_task, "tx_pipeline", 8192, NULL,
                    configMAX_PRIORITIES-1, &s_tx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX pipeline task");
        return KROSCHUTHREAD_STATUS_ERROR;
    }
    // Prebudíme hneď na štarte
    xTaskNotifyGive(s_tx_task_handle);

    g_protocol_state.initialized = true;
    ESP_LOGI(TAG, "Protocol ready (ISR pipeline active)"
#if !KROSCHUTHREAD_RECEIVER_ENABLED
                  " [TX-only]"
#endif
    );
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

// ===========================================================
// FRAME ENCAPSULATION
// ===========================================================
uint16_t kroschuthread_calculate_crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021; // CRC-16-CCITT poly
            else
                crc <<= 1;
        }
    }
    return crc;
}

kroschuthread_status_t kroschuthread_encapsulate_frame(
    const uint8_t* data,
    size_t data_length,
    kroschuthread_frame_type_t frame_type,
    uint16_t source_port,
    uint16_t destination_port,
    uint16_t sequence_number,
    kroschuthread_frame_t* frame)
{
    if (!data || !frame || data_length > KROSCHUTHREAD_MAX_PAYLOAD_SIZE)
        return KROSCHUTHREAD_STATUS_ERROR;

    frame->header.preamble          = KROSCHUTHREAD_FRAME_PREAMBLE;
    frame->header.sync              = KROSCHUTHREAD_FRAME_SYNC;
    frame->header.frame_type        = frame_type;
    frame->header.source_port       = source_port;
    frame->header.destination_port  = destination_port;
    frame->header.sequence_number   = sequence_number;
    frame->header.payload_length    = (uint8_t)data_length;
    frame->header.frame_length      = sizeof(kroschuthread_frame_header_t) + data_length + sizeof(uint16_t);

    memcpy(frame->payload, data, data_length);

    // compute CRC over header + payload
    size_t crc_input_len = sizeof(kroschuthread_frame_header_t) + data_length;
    frame->crc = kroschuthread_calculate_crc16((const uint8_t*)frame, crc_input_len);

    return KROSCHUTHREAD_STATUS_SUCCESS;
}

kroschuthread_status_t kroschuthread_protocol_send_data_no_ack(
    const uint8_t* data, size_t data_length, uint16_t destination_port)
{
    if (!g_protocol_state.initialized || !data || !data_length)
        return KROSCHUTHREAD_STATUS_ERROR;

    size_t offset = 0;
    while (offset < data_length) {
        size_t chunk_size = data_length - offset;
        if (chunk_size > KROSCHU_MAX_FRAGMENT_PAYLOAD)
            chunk_size = KROSCHU_MAX_FRAGMENT_PAYLOAD;

        kroschuthread_frame_t frame;
        uint16_t sequence = g_protocol_state.next_sequence_number++;

        // označ fragment typom FRAME_TYPE_DATA_FRAGMENT napr.
        if (kroschuthread_encapsulate_frame(
                &data[offset], chunk_size,
                KROSCHUTHREAD_FRAME_TYPE_DATA,
                KROSCHUTHREAD_DATA_PORT,
                destination_port, sequence, &frame) != KROSCHUTHREAD_STATUS_SUCCESS)
            return KROSCHUTHREAD_STATUS_ERROR;

        size_t total_frame_size = sizeof(kroschuthread_frame_header_t) + chunk_size + sizeof(uint16_t);

        kroschuthread_frame_queue_item_t item = {0};
        memcpy(item.frame_data, &frame, total_frame_size);
        item.frame_length = total_frame_size;

        if (xQueueSend(g_protocol_state.tx_queue, &item, 0) != pdTRUE)
            return KROSCHUTHREAD_STATUS_BUFFER_FULL;

        offset += chunk_size;
    }

    if (s_tx_task_handle) xTaskNotifyGive(s_tx_task_handle);
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

#if KROSCHUTHREAD_RECEIVER_ENABLED
#include "kroschuthread_radio.h"
kroschuthread_status_t kroschuthread_protocol_get_stats(kroschuthread_stats_t *stats)
{
    if (!stats)
        return KROSCHUTHREAD_STATUS_ERROR;

    extern kroschuthread_radio_state_t g_radio_state;

    stats->frames_transmitted = g_radio_state.stats.total_tx_frames;
    stats->frames_received    = g_radio_state.stats.total_rx_frames;
    stats->acks_sent          = 0;
    stats->acks_received      = 0;
    stats->crc_errors         = g_radio_state.stats.rx_crc_errors;
    stats->timeouts           = 0;
    stats->retransmissions    = 0;

    return KROSCHUTHREAD_STATUS_SUCCESS;
}
#endif
