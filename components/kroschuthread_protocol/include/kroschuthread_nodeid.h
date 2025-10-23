#ifndef KROSCHUTHREAD_NODEID_H
#define KROSCHUTHREAD_NODEID_H

#include <stdint.h>
#include <stdbool.h>

#define NODE_BROADCAST          0xFFFF
#define NODE_UNASSIGNED         0x0000
#define NODE_MAX_TABLE          64 
#define NODE_TIMEOUT_MS         (24ULL * 60ULL * 60ULL * 1000ULL)   // 24 h
#define NODE_SWEEP_INTERVAL     (60ULL * 1000ULL)                   // check one per min


typedef struct {
    uint16_t id;          
    int8_t   last_rssi;   
    uint16_t last_seq;    
    uint32_t last_seen_ms;
    uint32_t expire_ms;   
    uint8_t  hops;        
} node_entry_t;

typedef struct {
    uint16_t self_id; // own id
    node_entry_t table[NODE_MAX_TABLE];
    uint8_t count;
} node_context_t;


extern node_context_t g_node_ctx;

// === API ===
void nodeid_init_from_mac(void);
uint16_t nodeid_get(void);
bool nodeid_is_broadcast(uint16_t id);
bool nodeid_is_valid(uint16_t id);

void node_table_sweep_expired(void);
void node_table_init_after_boot(void);
void node_table_save_to_nvs(void);
void node_table_load_from_nvs(void);


// === Node table API ===
void node_table_add_or_update(uint16_t id, int8_t rssi);
node_entry_t* node_table_find(uint16_t id);
void node_table_debug_print(void);

#endif // KROSCHUTHREAD_NODEID_H
