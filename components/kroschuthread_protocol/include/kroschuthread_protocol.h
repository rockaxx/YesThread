#ifndef KROSCHUTHREAD_PROTOCOL_H
#define KROSCHUTHREAD_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Protocol Constants
#define KROSCHUTHREAD_PROTOCOL_VERSION      0x01
#define KROSCHUTHREAD_CHANNEL               20
#define KROSCHUTHREAD_DATA_PORT             1234
#define KROSCHUTHREAD_ACK_PORT              1235
#define KROSCHUTHREAD_MAX_PAYLOAD_SIZE      100
#define KROSCHUTHREAD_MAX_FRAME_SIZE        120
#define KROSCHUTHREAD_MAX_RETRIES           3
#define KROSCHUTHREAD_ACK_TIMEOUT_MS        50
#define KROSCHUTHREAD_FRAME_PREAMBLE        0xAB
#define KROSCHUTHREAD_FRAME_SYNC            0xCD
#define KROSCHU_MAX_FRAGMENT_PAYLOAD        90
#define KROSCHUTHREAD_TX_POWER              20  // dBm!!
#define KROSCHUTHREAD_RECEIVER_ENABLED 1

// Frame Types
typedef enum {
    KROSCHUTHREAD_FRAME_TYPE_DATA = 0x01,
    KROSCHUTHREAD_FRAME_TYPE_ACK = 0x02,
    KROSCHUTHREAD_FRAME_TYPE_FILE = 0x03,
} kroschuthread_frame_type_t;

// Frame Status
typedef enum {
    KROSCHUTHREAD_STATUS_SUCCESS = 0,
    KROSCHUTHREAD_STATUS_ERROR = -1,
    KROSCHUTHREAD_STATUS_TIMEOUT = -2,
    KROSCHUTHREAD_STATUS_CRC_ERROR = -3,
    KROSCHUTHREAD_STATUS_BUFFER_FULL = -4,
    KROSCHUTHREAD_STATUS_INVALID_FRAME = -5
} kroschuthread_status_t;

typedef struct __attribute__((packed)) {
    uint16_t preamble;
    uint16_t sync;
    uint8_t frame_type;
    uint16_t source_node;     
    uint16_t destination_node;
    uint16_t source_port;
    uint16_t destination_port;
    uint16_t sequence_number;
    uint16_t fragment_index;
    uint16_t total_fragments;
    uint8_t payload_length;
    uint16_t frame_length;
} kroschuthread_frame_header_t;


// Complete Frame Structure
typedef struct __attribute__((packed)) {
    kroschuthread_frame_header_t header;
    uint8_t payload[KROSCHUTHREAD_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} kroschuthread_frame_t;

// ACK Frame Structure (lightweight)
typedef struct __attribute__((packed)) {
    kroschuthread_frame_header_t header;
    uint16_t acked_sequence;
    uint16_t crc;
} kroschuthread_ack_frame_t;

// Callback function types
typedef void (*kroschuthread_data_received_callback_t)(const uint8_t* data, size_t data_length, uint16_t source_port);
typedef void (*kroschuthread_ack_received_callback_t)(uint16_t acked_sequence);

// Protocol Configuration
typedef struct {
    uint8_t channel;                                    // 802.15.4 channel
    int8_t tx_power;                                    // TX power
    kroschuthread_data_received_callback_t data_callback;
    kroschuthread_ack_received_callback_t ack_callback;
} kroschuthread_config_t;

// Protocol Statistics
typedef struct {
    uint32_t frames_transmitted;
    uint32_t frames_received;
    uint32_t acks_sent;
    uint32_t acks_received;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t retransmissions;
} kroschuthread_stats_t;

// Function Prototypes

/**
 * @brief Initialize the KroschuThread protocol
 * 
 * @param config Protocol configuration
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_init(const kroschuthread_config_t* config);

/**
 * @brief Deinitialize the protocol and free resources
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_deinit(void);

/**
 * @brief Send data to specified port with ACK confirmation
 * 
 * @param data Data buffer to send
 * @param data_length Length of data
 * @param destination_port Target port
 * @return kroschuthread_status_t Status code
 */

/**
 * @brief Send data without waiting for ACK (fire and forget)
 * 
 * @param data Data buffer to send
 * @param data_length Length of data  
 * @param destination_port Target port
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_send_data_no_ack(const uint8_t* data, size_t data_length, uint16_t dest_node, uint16_t destination_port);

/**
 * @brief Process incoming frames (call this in main loop)
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_process(void);

/**
 * @brief Get protocol statistics
 * 
 * @param stats Pointer to stats structure to fill
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_get_stats(kroschuthread_stats_t* stats);

/**
 * @brief Reset protocol statistics
 * 
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_protocol_reset_stats(void);

// Utility Functions

/**
 * @brief Calculate CRC-16 for data buffer
 * 
 * @param data Data buffer
 * @param length Data length
 * @return uint16_t CRC-16 value
 */
uint16_t kroschuthread_calculate_crc16(const uint8_t* data, size_t length);

/**
 * @brief Encapsulate data into protocol frame
 * 
 * @param data Source data
 * @param data_length Data length
 * @param frame_type Frame type
 * @param source_port Source port
 * @param destination_port Destination port
 * @param sequence_number Sequence number
 * @param frame Output frame buffer
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_encapsulate_frame(
    const uint8_t* data,
    size_t data_length,
    kroschuthread_frame_type_t frame_type,
    uint16_t source_port,
    uint16_t destination_port,
    uint16_t destination_node,
    uint16_t sequence_number,
    uint16_t fragment_index,
    uint16_t total_fragments,
    kroschuthread_frame_t* frame
);

/**
 * @brief Decapsulate protocol frame and extract data
 * 
 * @param frame Input frame
 * @param frame_length Frame length
 * @param data Output data buffer
 * @param data_length Output data length
 * @return kroschuthread_status_t Status code
 */
kroschuthread_status_t kroschuthread_decapsulate_frame(
    const uint8_t* frame, 
    size_t frame_length,
    uint8_t* data,
    size_t* data_length,
    kroschuthread_frame_header_t* header
);



#ifdef __cplusplus
}
#endif

#endif // KROSCHUTHREAD_PROTOCOL_H