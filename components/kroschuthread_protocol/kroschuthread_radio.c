#include "kroschuthread_radio.h"
#include "kroschuthread_nodeid.h"
#include <string.h>
#include <esp_log.h>
#include <esp_ieee802154.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static const char* TAG = "KroschuRadio";

// -----------------------------------------------------
// Internal radio state definition moved up
// -----------------------------------------------------

// Global state instance

kroschuthread_radio_state_t g_radio_state = {0};

// -----------------------------------------------------
// Optional TX callback
// -----------------------------------------------------
static void (*next_tx_ready_cb)(void) = NULL;
void kroschuthread_radio_set_next_tx_callback(void (*cb)(void))
{
    next_tx_ready_cb = cb;
}


// Forward declarations for ESP-IDF v5.5.1 interrupt handlers
#if KROSCHUTHREAD_RECEIVER_ENABLED
static void kroschuthread_radio_rx_done_handler(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info);
#endif
static void kroschuthread_radio_tx_done_handler(const uint8_t *frame, const uint8_t *ack, esp_ieee802154_frame_info_t *ack_frame_info);

kroschuthread_status_t kroschuthread_radio_init(const kroschuthread_radio_config_t* config)
{
    if (!config) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    if (g_radio_state.initialized) {
        ESP_LOGW(TAG, "Radio already initialized");
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    ESP_LOGI(TAG, "Initializing ESP32-C6 802.15.4 radio on channel %d", config->channel);

    // Validate channel
    if (config->channel < 11 || config->channel > 26) {
        ESP_LOGE(TAG, "Invalid channel %d, must be 11-26", config->channel);
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    // Copy configuration
    memcpy(&g_radio_state.config, config, sizeof(kroschuthread_radio_config_t));

    // Initialize statistics
    memset(&g_radio_state.stats, 0, sizeof(kroschuthread_radio_stats_t));
    g_radio_state.stats_mutex = xSemaphoreCreateMutex();
    if (!g_radio_state.stats_mutex) {
        ESP_LOGE(TAG, "Failed to create stats mutex");
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    // Now enable IEEE 802.15.4 subsystem (ISR hooks are linked via esp_ieee802154_*_done symbols)
    esp_err_t err = esp_ieee802154_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable IEEE 802.15.4: %s", esp_err_to_name(err));
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    // RX and promiscuous settings moved to performance optimization section

    // Get and set MAC address
    err = esp_read_mac(g_radio_state.local_mac_address, ESP_MAC_IEEE802154);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address: %s", esp_err_to_name(err));
        // Generate a simple MAC address based on default
        esp_efuse_mac_get_default(g_radio_state.local_mac_address);
        ESP_LOGW(TAG, "Using default MAC address");
    }

    // Configure addressing and PAN
    esp_ieee802154_set_extended_address(g_radio_state.local_mac_address);
    
    // Set PAN ID (use a default value)
    esp_ieee802154_set_panid(0x1234);

    // Set short address (use last 2 bytes of extended address)
    uint16_t short_addr = (g_radio_state.local_mac_address[6] << 8) | g_radio_state.local_mac_address[7];
    esp_ieee802154_set_short_address(short_addr);

    // Be explicit: non-coordinator, no auto pending
    esp_ieee802154_set_coordinator(false);
    esp_ieee802154_set_pending_mode(ESP_IEEE802154_AUTO_PENDING_DISABLE);
    // Set channel
    err = esp_ieee802154_set_channel(config->channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set channel: %s", esp_err_to_name(err));
        esp_ieee802154_disable();
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    // Set transmission power to maximum for speed/reliability
    err = esp_ieee802154_set_txpower(20); // Maximum power
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set TX power: %s", esp_err_to_name(err));
    }
    
    // PERFORMANCE OPTIMIZATIONS
    // Set radio to continuous RX mode for faster reception
    #if KROSCHUTHREAD_RECEIVER_ENABLED
        esp_ieee802154_set_rx_when_idle(true);
        
        // Disable frame filtering for maximum speed (accept all frames)
        esp_ieee802154_set_promiscuous(true);
        esp_ieee802154_receive();
    #else
        esp_ieee802154_set_rx_when_idle(false);
        
        // Disable frame filtering for maximum speed (accept all frames)
        esp_ieee802154_set_promiscuous(false);
    #endif
    // Use highest priority interrupt for 802.15.4
    // (Note: ACK control is done via FCF in frame, not API calls)

    // Keep promiscuous enabled (already set above) to accept broadcast frames

    g_radio_state.initialized = true;
    g_radio_state.rx_enabled = false;

    ESP_LOGI(TAG, "Radio initialized successfully");
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             g_radio_state.local_mac_address[0], g_radio_state.local_mac_address[1],
             g_radio_state.local_mac_address[2], g_radio_state.local_mac_address[3],
             g_radio_state.local_mac_address[4], g_radio_state.local_mac_address[5],
             g_radio_state.local_mac_address[6], g_radio_state.local_mac_address[7]);
    // Start reception
    return 0;
}

kroschuthread_status_t kroschuthread_radio_deinit(void)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    ESP_LOGI(TAG, "Deinitializing radio");

    // Stop reception
    kroschuthread_radio_stop_rx();

    // Disable IEEE 802.15.4
    esp_ieee802154_disable();

    // Clean up mutex
    if (g_radio_state.stats_mutex) {
        vSemaphoreDelete(g_radio_state.stats_mutex);
    }

    // Reset state
    memset(&g_radio_state, 0, sizeof(g_radio_state));

    ESP_LOGI(TAG, "Radio deinitialized");
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

kroschuthread_status_t kroschuthread_radio_transmit(const uint8_t* frame_data, size_t frame_length, uint16_t dest_node)
{
    if (!g_radio_state.initialized || !frame_data || frame_length == 0 || frame_length > 127)
        return KROSCHUTHREAD_STATUS_ERROR;

    if (g_radio_state.tx_in_progress) {
        return KROSCHUTHREAD_STATUS_BUFFER_FULL; // nech vyššia vrstva zaradí do fronty
    }
    g_radio_state.tx_in_progress = true;


    // For ESP-IDF v5.5.1, we need to create a proper 802.15.4 frame
    // Frame format: [Length][FCF][Seq][PAN_ID][Dest_Addr][Src_Addr][Payload][FCS]
    uint8_t *tx_frame = g_radio_state.tx_frame_buffer;
    uint8_t frame_idx = 0;
    
    // MINIMAL header for maximum speed
    const uint8_t mhr_len = 9; // FCF(2) + Seq(1) + PAN(2) + Dest(2) + Src(2)
    uint8_t total_len = mhr_len + (uint8_t)frame_length;
    tx_frame[frame_idx++] = total_len;
    
    // Frame Control Field (FCF) - OPTIMIZED for speed
    // No ACK requested, intra-PAN, short addressing
    // Byte0: FrameType=001, Security=0, Pending=0, AckReq=0, PanIDCompression=1 -> 0x41
    // Byte1: DestAddrMode=10 (short), FrameVersion=00, SrcAddrMode=10 (short) -> 0x88  
    tx_frame[frame_idx++] = 0x41; // Fast transmission, no ACK
    tx_frame[frame_idx++] = 0x88; // Short addressing
    
    // Sequence number
    tx_frame[frame_idx++] = (uint8_t)(g_radio_state.stats.total_tx_frames & 0xFF);
    
    // PAN ID (same for source and dest due to intra-PAN)
    tx_frame[frame_idx++] = 0x34; // PAN ID low byte
    tx_frame[frame_idx++] = 0x12; // PAN ID high byte
    
    // Destination address (short) - broadcast
    tx_frame[frame_idx++] = (uint8_t)(dest_node & 0xFF);
    tx_frame[frame_idx++] = (uint8_t)((dest_node >> 8) & 0xFF);
    
    // Source address (short)
    uint16_t src = nodeid_get();
    tx_frame[frame_idx++] = (uint8_t)(src & 0xFF);
    tx_frame[frame_idx++] = (uint8_t)(src >> 8);
        
    // Payload
    memcpy(&tx_frame[frame_idx], frame_data, frame_length);
    frame_idx += frame_length;
    
    esp_err_t err = esp_ieee802154_transmit(tx_frame, false);
    if (err != ESP_OK) {
        g_radio_state.tx_in_progress = false;
        return KROSCHUTHREAD_STATUS_ERROR;
    }
    g_radio_state.stats.total_tx_frames++;
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

kroschuthread_status_t kroschuthread_radio_set_channel(uint8_t channel)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    if (channel < 11 || channel > 26) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    esp_err_t err = esp_ieee802154_set_channel(channel);
    if (err == ESP_OK) {
        g_radio_state.config.channel = channel;
        ESP_LOGI(TAG, "Channel set to %d", channel);
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    ESP_LOGE(TAG, "Failed to set channel: %s", esp_err_to_name(err));
    return KROSCHUTHREAD_STATUS_ERROR;
}

kroschuthread_status_t kroschuthread_radio_set_tx_power(int8_t tx_power)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    esp_err_t err = esp_ieee802154_set_txpower(tx_power);
    if (err == ESP_OK) {
        g_radio_state.config.tx_power = tx_power;
        ESP_LOGI(TAG, "TX power set to %d dBm", tx_power);
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    ESP_LOGW(TAG, "Failed to set TX power: %s", esp_err_to_name(err));
    return KROSCHUTHREAD_STATUS_ERROR;
}

#if KROSCHUTHREAD_RECEIVER_ENABLED
kroschuthread_status_t kroschuthread_radio_start_rx(void)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    if (g_radio_state.rx_enabled) {
        return KROSCHUTHREAD_STATUS_SUCCESS; // Already enabled
    }

    esp_err_t err = esp_ieee802154_receive();
    if (err == ESP_OK) {
        g_radio_state.rx_enabled = true;
        ESP_LOGI(TAG, "Radio reception started on channel %d", g_radio_state.config.channel);
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    ESP_LOGE(TAG, "Failed to start reception: %s", esp_err_to_name(err));
    return KROSCHUTHREAD_STATUS_ERROR;
}

kroschuthread_status_t kroschuthread_radio_stop_rx(void)
{
    if (!g_radio_state.initialized || !g_radio_state.rx_enabled) {
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    // In ESP-IDF v5.5.1, we don't need to manually stop RX
    g_radio_state.rx_enabled = false;
    ESP_LOGD(TAG, "Radio reception stopped");
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

kroschuthread_status_t kroschuthread_radio_get_stats(kroschuthread_radio_stats_t* stats)
{
    if (!stats || !g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    if (xSemaphoreTake(g_radio_state.stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(stats, &g_radio_state.stats, sizeof(kroschuthread_radio_stats_t));
        xSemaphoreGive(g_radio_state.stats_mutex);
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    return KROSCHUTHREAD_STATUS_ERROR;
}
#endif

kroschuthread_status_t kroschuthread_radio_reset_stats(void)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    if (xSemaphoreTake(g_radio_state.stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&g_radio_state.stats, 0, sizeof(kroschuthread_radio_stats_t));
        xSemaphoreGive(g_radio_state.stats_mutex);
        return KROSCHUTHREAD_STATUS_SUCCESS;
    }

    return KROSCHUTHREAD_STATUS_ERROR;
}

kroschuthread_status_t kroschuthread_radio_set_promiscuous(bool enable)
{
    if (!g_radio_state.initialized) {
        return KROSCHUTHREAD_STATUS_ERROR;
    }

    esp_ieee802154_set_promiscuous(enable);
    ESP_LOGI(TAG, "Promiscuous mode %s", enable ? "enabled" : "disabled");
    return KROSCHUTHREAD_STATUS_SUCCESS;
}

int8_t kroschuthread_radio_get_rssi(void)
{
    if (!g_radio_state.initialized) {
        return -128; // Invalid RSSI
    }

    return g_radio_state.stats.last_rssi;
}

#if KROSCHUTHREAD_RECEIVER_ENABLED
static void kroschuthread_radio_rx_done_handler(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info)
{
    if (!g_radio_state.initialized || !frame) return;

    // TX-only? Neobnovuj RX
    if (!g_radio_state.config.rx_callback) {
        esp_ieee802154_receive_handle_done(frame);
        return;
    }

    g_radio_state.stats.total_rx_frames++;
    if (frame_info) {
        g_radio_state.stats.last_rssi = frame_info->rssi;
        g_radio_state.stats.last_lqi = frame_info->lqi;
    }

    // Parse 802.15.4 frame
    uint8_t rx_len = frame[0];
    
    // Note: Cannot use ESP_LOG functions in ISR - causes bootloop
    
    if (rx_len >= 5 && rx_len <= 127) {
        // Compute MHR length dynamically from FCF
        uint8_t fcf0 = frame[1];
        uint8_t fcf1 = frame[2];
        bool pan_comp = (fcf0 & 0x40) != 0;                  // PAN ID compression
        bool security = (fcf0 & 0x08) != 0;                  // security enabled
        uint8_t dest_mode = (fcf1 >> 2) & 0x03;             // 0:none, 2:short, 3:extended
        uint8_t frame_ver = (fcf1 >> 4) & 0x03;             // not used
        uint8_t src_mode  = (fcf1 >> 6) & 0x03;             // 0:none, 2:short, 3:extended
        (void)frame_ver;

        int mhr_len = 3; // FCF(2) + Seq(1)
        // Dest fields
        if (dest_mode != 0) {
            mhr_len += 2; // Dest PAN ID
            mhr_len += (dest_mode == 2) ? 2 : (dest_mode == 3) ? 8 : 0;
        }
        // Src fields
        if (src_mode != 0) {
            if (!pan_comp) {
                mhr_len += 2; // Src PAN ID if no compression
            }
            mhr_len += (src_mode == 2) ? 2 : (src_mode == 3) ? 8 : 0;
        }
        // Aux security header (not used here)
        if (security) {
            // Conservative skip of 5 bytes if security bit somehow set (Frame Counter + Control)
            mhr_len += 5;
        }

        if (mhr_len < 3 || (1 + mhr_len) > 127) {
            g_radio_state.stats.rx_crc_errors++;
            goto done_rearm;
        }

        int payload_start_idx = 1 + mhr_len;
        int payload_len       = (int)rx_len - mhr_len; 

        if (payload_len > 0 && payload_len <= 120 && payload_start_idx + payload_len <= 128) {
            const uint8_t *payload = &frame[payload_start_idx];
            if (g_radio_state.config.rx_callback) {
                g_radio_state.config.rx_callback(payload, (size_t)payload_len);
            }
        }
        else {
            g_radio_state.stats.rx_crc_errors++;
        }
    } else {
        g_radio_state.stats.rx_crc_errors++;
    }

done_rearm:

    esp_ieee802154_receive_handle_done(frame);
    esp_ieee802154_receive();
}
#endif



// === TX done handler – len callback ===
static void kroschuthread_radio_tx_done_handler(const uint8_t *frame,
                                                const uint8_t *ack,
                                                esp_ieee802154_frame_info_t *ack_frame_info)
{
    (void)frame; (void)ack; (void)ack_frame_info;

    g_radio_state.tx_in_progress = false;

#if KROSCHUTHREAD_RECEIVER_ENABLED

    esp_ieee802154_receive();
    g_radio_state.rx_enabled = true;
#endif

    if (next_tx_ready_cb)
        next_tx_ready_cb();
}


void IRAM_ATTR esp_ieee802154_transmit_done(const uint8_t *frame,
                                            const uint8_t *ack,
                                            esp_ieee802154_frame_info_t *ack_frame_info)
{
    kroschuthread_radio_tx_done_handler(frame, ack, ack_frame_info);
}

void IRAM_ATTR esp_ieee802154_transmit_failed(const uint8_t *frame,
                                              esp_ieee802154_tx_error_t error)
{
    (void)frame; (void)error;
    g_radio_state.tx_in_progress = false;
    if (next_tx_ready_cb) next_tx_ready_cb(); // skúsi ďalší
}
#if KROSCHUTHREAD_RECEIVER_ENABLED
void IRAM_ATTR esp_ieee802154_receive_done(uint8_t *frame,
                                        esp_ieee802154_frame_info_t *frame_info)
{
    kroschuthread_radio_rx_done_handler(frame, frame_info);
}
#endif