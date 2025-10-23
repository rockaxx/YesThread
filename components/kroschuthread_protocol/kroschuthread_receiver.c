#include "kroschuthread_radio.h"
#include "kroschuthread_nodeid.h"
#include "kroschuthread_protocol.h"   // 👈 Dôležité! pre definíciu frame štruktúry
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "KroschuReceiver"

#if KROSCHUTHREAD_RECEIVER_ENABLED

static void receiver_rx_callback(const uint8_t *frame, size_t frame_length)
{
    const kroschuthread_frame_t *parsed = (const kroschuthread_frame_t *)frame;

    if (parsed->header.destination_node == nodeid_get() ||
        parsed->header.destination_node == NODE_BROADCAST)
    {
        ESP_LOGI(TAG, "RX Frame (%d bytes) from 0x%04X -> 0x%04X",
                 (int)frame_length,
                 parsed->header.source_node,
                 parsed->header.destination_node);

        // Výpis len čistého payloadu
        size_t payload_len = parsed->header.payload_length;
        printf("Payload (ASCII): ");
        for (size_t i = 0; i < payload_len; i++) {
            char c = parsed->payload[i];
            putchar((c >= 32 && c <= 126) ? c : '.');
        }
        printf("\n");
    }
}


void receiver_task(void *arg)
{
    ESP_LOGI(TAG, "Receiver task started (RX mode)");

    kroschuthread_radio_config_t rx_config = {
        .channel = KROSCHUTHREAD_CHANNEL,
        .tx_power = 8,
        .rx_callback = receiver_rx_callback
    };

    if (kroschuthread_radio_init(&rx_config) != KROSCHUTHREAD_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to init radio for RX");
        vTaskDelete(NULL);
        return;
    }

    esp_ieee802154_set_promiscuous(true);
    esp_ieee802154_set_rx_when_idle(true);
    esp_ieee802154_receive();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // KROSCHUTHREAD_RECEIVER_ENABLED
