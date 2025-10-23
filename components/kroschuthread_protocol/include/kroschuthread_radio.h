#ifndef KROSCHUTHREAD_RADIO_H
#define KROSCHUTHREAD_RADIO_H

#define KROSCHUTHREAD_RECEIVER_ENABLED 1  // 0 = TX only, 1 = enable RX/TX mode

#include "kroschuthread_protocol.h"
#include <stdint.h>
#include <stddef.h>
#include <esp_ieee802154_types.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h"   

#ifdef __cplusplus
extern "C" {
#endif

// Radio-specific configuration
typedef struct {
    uint8_t channel;                                    // 802.15.4 channel (11-26)
    int8_t tx_power;                                    // Transmission power in dBm
    void (*rx_callback)(const uint8_t* frame, size_t frame_length);  // RX callback function
} kroschuthread_radio_config_t;
// =====================================================
// Radio statistics structure
// =====================================================


typedef struct {
    uint32_t total_tx_frames;
    uint32_t total_rx_frames;
    uint32_t acks_sent;
    uint32_t acks_received;
    uint32_t rx_crc_errors;
    int8_t   last_rssi;      // ⬅️ doplnené
    uint8_t  last_lqi;       // ⬅️ doplnené
} kroschuthread_radio_stats_t;


typedef struct {
    bool initialized;
    kroschuthread_radio_config_t config;
    kroschuthread_radio_stats_t stats;
    SemaphoreHandle_t stats_mutex;
    uint8_t local_mac_address[8];
    bool rx_enabled;
    volatile bool tx_in_progress;
    uint8_t tx_frame_buffer[128];
} kroschuthread_radio_state_t;


extern kroschuthread_radio_state_t g_radio_state;


/**
 * @brief Initialize the ESP32-C6 802.15.4 radio
 * 
 * @param config Radio configuration
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_init(const kroschuthread_radio_config_t* config);

/**
 * @brief Deinitialize the radio and free resources
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_deinit(void);

/**
 * @brief Transmit a frame over 802.15.4 radio
 * 
 * @param frame_data Frame data to transmit
 * @param frame_length Length of frame data
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_transmit(const uint8_t* frame_data, size_t frame_length, uint16_t dest_node);

/**
 * @brief Set radio channel
 * 
 * @param channel 802.15.4 channel (11-26)
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_set_channel(uint8_t channel);

/**
 * @brief Set transmission power
 * 
 * @param tx_power Power in dBm
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_set_tx_power(int8_t tx_power);

/**
 * @brief Get radio statistics
 * 
 * @param stats Pointer to stats structure to fill
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_get_stats(kroschuthread_radio_stats_t* stats);

/**
 * @brief Reset radio statistics
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_reset_stats(void);

/**
 * @brief Enable/disable radio promiscuous mode
 * 
 * @param enable True to enable, false to disable
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_set_promiscuous(bool enable);

/**
 * @brief Get current RSSI value
 * 
 * @return int8_t RSSI value in dBm
 */
int8_t kroschuthread_radio_get_rssi(void);

/**
 * @brief Start radio reception
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_start_rx(void);

/**
 * @brief Stop radio reception
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_radio_stop_rx(void);
void kroschuthread_radio_set_next_tx_callback(void (*cb)(void));


#ifdef __cplusplus
}
#endif

#endif // KROSCHUTHREAD_RADIO_H