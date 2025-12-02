#include "kroschuthread_nodeid.h"
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_log.h>
#include <string.h>
#include <sys/time.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "NodeID";

node_context_t g_node_ctx = {
    .self_id = NODE_UNASSIGNED,
    .count   = 0
};

static uint32_t millis(void)
{
    return esp_timer_get_time() / 1000ULL;
}


static uint16_t simple_mac_hash(const uint8_t mac[8])
{
    uint16_t h = mac[0];
    for (int i = 1; i < 8; i++)
        h = (h << 3) ^ mac[i] ^ (h >> 5);
    h = (h & 0xFFFE) | ((h == 0) ? 1 : 0);
    return h;
}

void nodeid_init_from_mac(void)
{
    uint8_t mac[8];
    esp_read_mac(mac, ESP_MAC_IEEE802154);
    g_node_ctx.self_id = simple_mac_hash(mac);
    g_node_ctx.count = 0;

    ESP_LOGI(TAG, "Initialized NodeID 0x%04X from IEEE802.15.4 MAC %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             g_node_ctx.self_id, mac[0], mac[1], mac[2], mac[3],
             mac[4], mac[5], mac[6], mac[7]);
}

uint16_t nodeid_get(void)
{
    return g_node_ctx.self_id;
}

bool nodeid_is_broadcast(uint16_t id)
{
    return id == NODE_BROADCAST;
}

bool nodeid_is_valid(uint16_t id)
{
    return (id != NODE_UNASSIGNED && id != NODE_BROADCAST);
}


node_entry_t* node_table_find(uint16_t id)
{
    for (int i = 0; i < g_node_ctx.count; i++)
        if (g_node_ctx.table[i].id == id)
            return &g_node_ctx.table[i];
    return NULL;
}

void node_table_add_or_update(uint16_t id, int8_t rssi)
{
    if (!nodeid_is_valid(id) || id == g_node_ctx.self_id)
        return;

    uint32_t now = millis();
    node_entry_t* n = node_table_find(id);

    if (n) {
        n->last_rssi = rssi;
        n->last_seen_ms = now;
        n->expire_ms = now + NODE_TIMEOUT_MS;
        return;
    }

    if (g_node_ctx.count < NODE_MAX_TABLE) {
        node_entry_t* slot = &g_node_ctx.table[g_node_ctx.count++];
        slot->id = id;
        slot->last_rssi = rssi;
        slot->last_seq = 0;
        slot->last_frag = 0xFFFF;
        slot->last_seen_ms = now;
        slot->expire_ms = now + NODE_TIMEOUT_MS;
        slot->hops = 1;
        ESP_LOGI(TAG, "Discovered new node: 0x%04X (RSSI %d)", id, rssi);
    }
}

void node_table_sweep_expired(void)
{
    uint32_t now = millis();
    for (int i = 0; i < g_node_ctx.count; i++) {
        node_entry_t* n = &g_node_ctx.table[i];
        if (now > n->expire_ms) {
            ESP_LOGI(TAG, "Node 0x%04X expired (last seen %lu ms ago)",
                     n->id, (unsigned long)(now - n->last_seen_ms));

            // vymaž posunutím zvyšku poľa
            memmove(&g_node_ctx.table[i], &g_node_ctx.table[i + 1],
                    (g_node_ctx.count - i - 1) * sizeof(node_entry_t));
            g_node_ctx.count--;
            i--;
        }
    }
}


void node_table_debug_print(void)
{
    ESP_LOGI(TAG, "=== NODE TABLE (%d entries) ===", g_node_ctx.count);
    for (int i = 0; i < g_node_ctx.count; i++) {
        node_entry_t* n = &g_node_ctx.table[i];
        ESP_LOGI(TAG, "ID=0x%04X  RSSI=%d  last_seen=%lu ms",
                 n->id, n->last_rssi, (unsigned long)n->last_seen_ms);
    }
}


void node_table_save_to_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("mesh", NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_blob(h, "router_tbl", g_node_ctx.table,
                 g_node_ctx.count * sizeof(node_entry_t));
    nvs_set_u8(h, "router_cnt", g_node_ctx.count);
    nvs_commit(h);
    nvs_close(h);
}

void node_table_load_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("mesh", NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No NVS partition or 'mesh' namespace yet (fresh boot)");
        return;
    }

    uint8_t cnt = 0;
    err = nvs_get_u8(h, "router_cnt", &cnt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No router table found in NVS (fresh start)");
        nvs_close(h);
        return;
    }

    if (cnt > NODE_MAX_TABLE) cnt = NODE_MAX_TABLE;
    size_t len = cnt * sizeof(node_entry_t);
    err = nvs_get_blob(h, "router_tbl", g_node_ctx.table, &len);
    nvs_close(h);

    if (err == ESP_OK) {
        g_node_ctx.count = cnt;
        ESP_LOGI(TAG, "Loaded %d cached routers", g_node_ctx.count);
    } else {
        ESP_LOGW(TAG, "Router table blob not found (fresh device)");
    }
}



static void router_sweep_task(void* arg)
{
    for (;;) {
        node_table_sweep_expired();
        vTaskDelay(pdMS_TO_TICKS(NODE_SWEEP_INTERVAL));
    }
}

void node_table_init_after_boot(void)
{
    node_table_load_from_nvs();

    // spusti údržbu ako FreeRTOS task
    xTaskCreatePinnedToCore(
        router_sweep_task,     // funkcia
        "router_sweep",        // názov úlohy
        4096,                  // stack
        NULL,                  // parameter
        3,                     // priorita
        NULL,                  // handle
        0                      // core 0 (stačí)
    );
}
