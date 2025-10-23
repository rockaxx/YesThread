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
#include "kroschuthread_nodeid.h"
static const char* TAG = "KroschuThreadMain";

// Demo mode selection - change this to test different modes
#define DEMO_MODE_SENDER     1
#define DEMO_MODE_RECEIVER   2
#define DEMO_MODE DEMO_MODE_SENDER  // Change to DEMO_MODE_RECEIVER for receiver

// Callback functions
void data_received_callback(const uint8_t* data, size_t data_length, uint16_t source_port)
{
    // ASCII výpis prijatého obsahu
    printf("Received: ");
    for (size_t i = 0; i < data_length; i++) {
        char c = (char)data[i];
        putchar((c >= 32 && c <= 126) ? c : '.');
    }
    printf("\n");

}

void mesh_debug_task(void) {

    node_table_debug_print();

}

void ack_received_callback(uint16_t acked_sequence)
{
    ESP_LOGI(TAG, "ACK received for sequence %d", acked_sequence);
}

void sender_task(void* parameters)
{
    ESP_LOGI(TAG, "Starting sender demo...");
    
    // Initialize protocol
    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = 20,  // Maximum power for best range
        .data_callback = data_received_callback,
        .ack_callback = ack_received_callback
    };
    
    kroschuthread_status_t status = kroschuthread_protocol_init(&config);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %d", status);
        vTaskDelete(NULL);
        return;
    }
        
    ESP_LOGI(TAG, "=== KROSCHUTHREAD FRAGMENT TEST ===");

    uint32_t message_counter = 0;

    const char *long_test_message =
    "KASANOVAPICA:abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    while (1) {
        message_counter++;


        status = kroschuthread_protocol_send_data_no_ack(
            (const uint8_t *)long_test_message,
            strlen(long_test_message),
            0x8A0E,
            1234
        );

        ESP_LOGI(TAG, "TX #%u status=%d len=%d", message_counter, status, strlen(long_test_message));

        vTaskDelay(pdMS_TO_TICKS(5000)); // 0.5 s pauza medzi burstami
        mesh_debug_task();
    }


}

void receiver_task(void* parameters)
{
    ESP_LOGI(TAG, "Starting receiver demo...");
    
    // Initialize protocol
    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = KROSCHUTHREAD_TX_POWER,
        .data_callback = data_received_callback,
        .ack_callback = ack_received_callback
    };
    
    kroschuthread_status_t status = kroschuthread_protocol_init(&config);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %d", status);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Protocol initialized successfully - waiting for messages...");
    
    uint32_t stats_counter = 0;
    
    while (1) {
        // Print statistics every 30 seconds
        if (++stats_counter % 30 == 0) {
            kroschuthread_stats_t stats;
            if (kroschuthread_protocol_get_stats(&stats) == KROSCHUTHREAD_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Stats - TX: %" PRIu32 ", RX: %" PRIu32 ", ACKs sent: %" PRIu32 ", ACKs received: %" PRIu32 ", CRC errors: %" PRIu32,
                         stats.frames_transmitted, stats.frames_received,
                         stats.acks_sent, stats.acks_received,
                         stats.crc_errors);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}




void app_main(void)
{
    ESP_LOGI(TAG, "KroschuThread Protocol Demo Starting...");
    mesh_debug_task();
    // Initialize NVS for PHY calibration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Print build configuration
    ESP_LOGI(TAG, "ESP32-C6 802.15.4 Custom Protocol");
    ESP_LOGI(TAG, "Channel: %d", KROSCHUTHREAD_CHANNEL);
    ESP_LOGI(TAG, "Data Port: %d", KROSCHUTHREAD_DATA_PORT);
    ESP_LOGI(TAG, "ACK Port: %d", KROSCHUTHREAD_ACK_PORT);
    ESP_LOGI(TAG, "Max Payload: %d bytes", KROSCHUTHREAD_MAX_PAYLOAD_SIZE);
    
#if DEMO_MODE == DEMO_MODE_SENDER
    ESP_LOGI(TAG, "Running in SENDER mode");
    xTaskCreate(sender_task, "sender_task", 8192, NULL, 5, NULL);
#elif DEMO_MODE == DEMO_MODE_RECEIVER
    ESP_LOGI(TAG, "Running in RECEIVER mode");
    xTaskCreate(receiver_task, "receiver_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGE(TAG, "Invalid demo mode configured!");
#endif
}
