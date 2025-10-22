#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include "kroschuthread_protocol.h"
#include "kroschuthread_radio.h"


static const char *TAG = "KroschuReceiverMain";

#define DEMO_MODE_RECEIVER 1
#define DEMO_MODE DEMO_MODE_RECEIVER
int dtrcv=0;

// =====================================================
// RX CALLBACK – spracovanie prijatých dát
// =====================================================
static void data_received_callback(const uint8_t *data, size_t data_length, uint16_t source_port)
{
    if (!data || data_length == 0) return;

    ESP_LOGI(TAG, "=== DATA RECEIVED ===");
    ESP_LOGI(TAG, "From port %u, %u bytes", (unsigned)source_port, (unsigned)data_length);
    
    dtrcv++;
    ESP_LOGI(TAG, "=== DATA COUNTED: %d ===",dtrcv);

    // HEX dump po 16 bajtov bezpečne
    char hex_line[64];
    for (size_t i = 0; i < data_length; i += 16) {
        size_t off = 0;
        memset(hex_line, 0, sizeof(hex_line));
        for (size_t j = 0; j < 16 && (i + j) < data_length; ++j) {
            off += snprintf(hex_line + off, sizeof(hex_line) - off, "%02X ", data[i + j]);
            if (off >= sizeof(hex_line)) break;
        }
        ESP_LOGI(TAG, "DATA[%03u]: %s", (unsigned)i, hex_line);
    }
}


// =====================================================
// ACK CALLBACK (ak by sa používal)
// =====================================================
static void ack_received_callback(uint16_t acked_sequence)
{
    ESP_LOGI(TAG, "ACK received for sequence %u", acked_sequence);
}

// =====================================================
// RECEIVER TASK
// =====================================================
void receiver_task(void *parameters)
{
    ESP_LOGI(TAG, "Starting receiver task...");

    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = 20, // receiver nepotrebuje vysoký výkon
        .data_callback = data_received_callback,
        .ack_callback = ack_received_callback
    };

    kroschuthread_status_t status = kroschuthread_protocol_init(&config);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to initialize protocol: %d", status);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Protocol initialized successfully - waiting for messages...");

    while (1)
    {
        // voliteľné: vypíš štatistiky každých 10 sekúnd
        static uint32_t counter = 0;
        if (++counter % 10 == 0)
        {
            kroschuthread_stats_t stats;
            if (kroschuthread_protocol_get_stats(&stats) == KROSCHUTHREAD_STATUS_SUCCESS)
            {
                ESP_LOGI(TAG, "Stats - RX: %" PRIu32 ", CRC errors: %" PRIu32,
                         stats.frames_received, stats.crc_errors);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =====================================================
// MAIN ENTRY POINT
// =====================================================
void app_main(void)
{
    ESP_LOGI(TAG, "=== KroschuThread Receiver Demo Starting ===");

    // Inicializácia NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");

    // Info log
    ESP_LOGI(TAG, "ESP32-C6 802.15.4 Receiver");
    ESP_LOGI(TAG, "Channel: %d", KROSCHUTHREAD_CHANNEL);
    ESP_LOGI(TAG, "Data Port: %d", KROSCHUTHREAD_DATA_PORT);
    ESP_LOGI(TAG, "ACK Port: %d", KROSCHUTHREAD_ACK_PORT);

    // Spusti receiver task
    xTaskCreate(receiver_task, "receiver_task", 4096, NULL, 5, NULL);
}
