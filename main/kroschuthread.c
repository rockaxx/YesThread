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

static const char* TAG = "KroschuThreadMain";

// Demo mode selection - change this to test different modes
#define DEMO_MODE_SENDER     1
#define DEMO_MODE_RECEIVER   2
#define DEMO_MODE DEMO_MODE_SENDER  // Change to DEMO_MODE_RECEIVER for receiver

// Callback functions
void data_received_callback(const uint8_t* data, size_t data_length, uint16_t source_port)
{
    ESP_LOGI(TAG, "=== DATA RECEIVED ===");
    ESP_LOGI(TAG, "From port %d: %.*s (%d bytes)", source_port, (int)data_length, data, (int)data_length);
    
    // Hex dump of received data
    ESP_LOGI(TAG, "Raw data hex:");
    for (int i = 0; i < (int)data_length && i < 64; i += 16) {
        char hex_line[64] = {0};
        for (int j = 0; j < 16 && (i + j) < (int)data_length && (i + j) < 64; j++) {
            sprintf(hex_line + j * 3, "%02X ", data[i + j]);
        }
        ESP_LOGI(TAG, "DATA[%03d]: %s", i, hex_line);
    }
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
    
    ESP_LOGI(TAG, "Protocol initialized successfully");
    
    uint32_t message_counter = 0;
    char message_buffer[64];
    
    // Speed measurement
    uint32_t start_time = xTaskGetTickCount();
    uint32_t last_stats_time = start_time;
    
    while (1) {
        // Send periodic messages
        //snprintf(message_buffer, sizeof(message_buffer), "Hello from KroschuThread #%" PRIu32, message_counter++);
        uint64_t ts_us = esp_timer_get_time();

        snprintf(message_buffer, sizeof(message_buffer),
                "1eiutxykhuvgxxykyeugczxgvwajsuahjicykfavahiyqulnabazccjcjiiaets1");

        // Sending logging removed for maximum speed
        
        status = kroschuthread_protocol_send_data(
            (uint8_t*)message_buffer, 
            strlen(message_buffer), 
            KROSCHUTHREAD_DATA_PORT
        );
        
        // TX result logging removed for speed
        
        // Stats logging removed for maximum speed
        
        // Measure and report speed every 1000 messages
        if (message_counter % 1000 == 0 && message_counter > 0) {
            uint32_t current_time = xTaskGetTickCount();
            uint32_t elapsed = current_time - last_stats_time;
            float messages_per_second = 1000.0f * 1000.0f / elapsed; // 1000 msgs / elapsed_ms
            ESP_LOGI(TAG, "SPEED: %d msgs, %.1f msg/sec, %.2f ms/msg", 
                     message_counter, messages_per_second, elapsed / 1000.0f);
            last_stats_time = current_time;
        }
        
        // No delay - send as fast as possible
    }
}

void receiver_task(void* parameters)
{
    ESP_LOGI(TAG, "Starting receiver demo...");
    
    // Initialize protocol
    kroschuthread_config_t config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = 20,
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
    xTaskCreate(sender_task, "sender_task", 4096, NULL, 5, NULL);
#elif DEMO_MODE == DEMO_MODE_RECEIVER
    ESP_LOGI(TAG, "Running in RECEIVER mode");
    xTaskCreate(receiver_task, "receiver_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGE(TAG, "Invalid demo mode configured!");
#endif
}
