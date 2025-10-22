#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kroschuthread_protocol.h"

static const char* TAG = "DebugMinimal";

static void data_received_callback(const uint8_t* data, size_t length, uint16_t source_port)
{
    ESP_LOGI(TAG, "=== DATA RECEIVED ===");
    ESP_LOGI(TAG, "Source port: %d, Length: %d", source_port, (int)length);
    
    // Hex dump of received data
    for (int i = 0; i < (int)length && i < 32; i += 16) {
        char hex_line[64] = {0};
        char ascii_line[17] = {0};
        for (int j = 0; j < 16 && (i + j) < (int)length && (i + j) < 32; j++) {
            sprintf(hex_line + j * 3, "%02X ", data[i + j]);
            ascii_line[j] = (data[i + j] >= 32 && data[i + j] < 127) ? data[i + j] : '.';
        }
        ESP_LOGI(TAG, "DATA[%03d]: %s| %s", i, hex_line, ascii_line);
    }
}

static void ack_received_callback(uint16_t acked_sequence)
{
    ESP_LOGI(TAG, "ACK received for sequence: %d", acked_sequence);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting KroschuThread minimal CRC debug test");
    
    // Initialize protocol
    kroschuthread_config_t config = {
        .channel = 23,
        .tx_power = 5,
        .data_callback = data_received_callback,
        .ack_callback = ack_received_callback
    };
    
    kroschuthread_status_t status = kroschuthread_protocol_init(&config);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %d", status);
        return;
    }
    
    ESP_LOGI(TAG, "Protocol initialized, waiting 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Send minimal payload - single byte
    uint8_t test_data[] = {0xAA};
    
    ESP_LOGI(TAG, "=== SENDING TEST DATA ===");
    ESP_LOGI(TAG, "Test payload: 0x%02X (length: %d)", test_data[0], sizeof(test_data));
    
    status = kroschuthread_protocol_send_data_no_ack(test_data, sizeof(test_data), 1234);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to send data: %d", status);
    } else {
        ESP_LOGI(TAG, "Data sent successfully");
    }
    
    // Send another test with 2 bytes
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint8_t test_data2[] = {0x55, 0xCC};
    
    ESP_LOGI(TAG, "=== SENDING TEST DATA 2 ===");
    ESP_LOGI(TAG, "Test payload: 0x%02X 0x%02X (length: %d)", test_data2[0], test_data2[1], sizeof(test_data2));
    
    status = kroschuthread_protocol_send_data_no_ack(test_data2, sizeof(test_data2), 5678);
    if (status != KROSCHUTHREAD_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to send data 2: %d", status);
    } else {
        ESP_LOGI(TAG, "Data 2 sent successfully");
    }
    
    // Keep listening
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        kroschuthread_stats_t stats;
        if (kroschuthread_protocol_get_stats(&stats) == KROSCHUTHREAD_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "=== STATS ===");
            ESP_LOGI(TAG, "TX: %d, RX: %d, CRC errors: %d, Timeouts: %d", 
                     (int)stats.frames_transmitted, (int)stats.frames_received, 
                     (int)stats.crc_errors, (int)stats.timeouts);
        }
    }
}