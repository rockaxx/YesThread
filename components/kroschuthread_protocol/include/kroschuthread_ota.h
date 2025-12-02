#pragma once
#include <stdint.h>
#include <stddef.h>

void kroschuthread_ota_process_message(const uint8_t *payload, size_t len);
void kroschuthread_ota_flash_fw(void);
void delete_ota_file(void);
void mount_spiffs();

#ifdef __cplusplus
extern "C" {
#endif

// Fragment reassembler API (volané z kroschuthread_protocol.c)
void reasm_add_fragment(const uint8_t *data, size_t len, uint16_t seq,
                        uint16_t frag_idx, uint16_t total_frags, uint16_t src_node);


#ifdef __cplusplus
}
#endif
