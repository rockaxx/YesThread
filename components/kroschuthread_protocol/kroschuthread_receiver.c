#include "kroschuthread_radio.h"
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "KroschuReceiver"

#if KROSCHUTHREAD_RECEIVER_ENABLED

static void receiver_rx_callback(const uint8_t *frame, size_t frame_length)
{
    if (frame_length == 0 || frame == NULL)
        return;

    ESP_LOGI(TAG, "RX Frame (%d bytes):", (int)frame_length);

    // Dump payload in hex
    for (size_t i = 0; i < frame_length; i++)
        printf("%02X ", frame[i]);
    printf("\n");
}

void receiver_task(void *arg)
{
    ESP_LOGI(TAG, "Receiver task started (RX mode)");

    // Konfigurácia rádia
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

    // Loop
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // KROSCHUTHREAD_RECEIVER_ENABLED
