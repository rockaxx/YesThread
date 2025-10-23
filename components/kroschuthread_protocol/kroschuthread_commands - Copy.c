#include "kroschuthread_commands.h"
#include "kroschuthread_protocol.h"
#include "kroschuthread_nodeid.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Select backend based on console configuration (USB Serial/JTAG vs UART0) ---
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  #define KROSCHU_BACKEND_USB 1
  #include "driver/usb_serial_jtag.h"
#else
  #define KROSCHU_BACKEND_USB 0
  #include "driver/uart.h"
#endif

static const char *TAG = "KroschuCmd";

// ====== Command definition ======
typedef void (*command_handler_t)(int argc, char **argv);
typedef struct {
    const char *name;
    const char *help;
    command_handler_t handler;
} command_t;

// ====== Forward declarations ======
static void cmd_sendto(int argc, char **argv);
static void cmd_commands(int argc, char **argv);
static void cmd_nodes(int argc, char **argv);

// ====== Command table ======
static const command_t g_commands[] = {
    { "sendto",   "sendto <node_id_hex> <message>  - Send a text message to target node", cmd_sendto },
    { "commands", "commands                         - List all available commands",       cmd_commands },
    { "nodes",    "nodes                            - Print known node table",            cmd_nodes },
};
static const int g_command_count = sizeof(g_commands) / sizeof(g_commands[0]);

// ====== Utils ======
static size_t join_args(char *dst, size_t dst_size, int argc, char **argv, int start_idx)
{
    size_t pos = 0;
    for (int i = start_idx; i < argc; i++) {
        const char *p = argv[i];
        while (*p && pos + 1 < dst_size) dst[pos++] = *p++;
        if (i < argc - 1 && pos + 1 < dst_size) dst[pos++] = ' ';
    }
    dst[pos] = '\0';
    return pos;
}

// ====== Command implementations ======
static void cmd_sendto(int argc, char **argv)
{
    if (argc < 3) {
        ESP_LOGW(TAG, "Usage: sendto <hex_addr> <message>");
        return;
    }
    uint16_t dest = (uint16_t)strtol(argv[1], NULL, 16);

    char msgbuf[KROSCHUTHREAD_MAX_PAYLOAD_SIZE + 1];
    size_t mlen = join_args(msgbuf, sizeof(msgbuf), argc, argv, 2);

    if (mlen == 0) {
        ESP_LOGW(TAG, "Message is empty");
        return;
    }

    ESP_LOGI(TAG, "Sending to node 0x%04X: '%s'", dest, msgbuf);

    kroschuthread_status_t st = kroschuthread_protocol_send_data_no_ack(
        (const uint8_t*)msgbuf, mlen, dest, KROSCHUTHREAD_DATA_PORT);

    if (st == KROSCHUTHREAD_STATUS_SUCCESS) ESP_LOGI(TAG, "Message sent");
    else ESP_LOGE(TAG, "Send failed (%d)", st);
}

static void cmd_commands(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("\n=== Available commands ===\n");
    for (int i = 0; i < g_command_count; i++) {
        printf("%-12s - %s\n", g_commands[i].name, g_commands[i].help);
    }
    printf("==========================\n\n");
}

static void cmd_nodes(int argc, char **argv)
{
    (void)argc; (void)argv;
    node_table_debug_print();
}

// ====== Parser ======
static void command_parse_and_execute(char *input)
{
    char *argv[16];
    int argc = 0;

    char *token = strtok(input, " \t\r\n");
    while (token && argc < (int)(sizeof(argv) / sizeof(argv[0]))) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }
    if (argc == 0) return;

    for (int i = 0; i < g_command_count; i++) {
        if (strcmp(argv[0], g_commands[i].name) == 0) {
            g_commands[i].handler(argc, argv);
            return;
        }
    }
    ESP_LOGW(TAG, "Unknown command: %s", argv[0]);
}

// ====== Backend I/O primitives ======
#if KROSCHU_BACKEND_USB
// --- USB Serial/JTAG (needs driver_install before first read/write) ---
static int s_usb_ready = 0;

static int io_init(void) {
    printf("[CLI] Backend: USB Serial/JTAG\r\n");

    // In IDF 5.5.x tx_buffer_size MUST be > 0
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        // INVALID_STATE = already installed by someone else -> OK
        s_usb_ready = 1;
        return 0;
    }
    ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
    return -1;
}

static int io_read_byte(uint8_t *c) {
    if (!s_usb_ready) return 0;
    int n = usb_serial_jtag_read_bytes(c, 1, 10);
    return (n == 1) ? 1 : 0;
}
static void io_write_str(const char *s) {
    if (!s_usb_ready) return;
    usb_serial_jtag_write_bytes((const uint8_t*)s, strlen(s), 20);
}
static void io_putc(int c) {
    if (!s_usb_ready) return;
    uint8_t b = (uint8_t)c;
    usb_serial_jtag_write_bytes(&b, 1, 20);
}

#else
// --- UART0 ---
#define CMD_UART_PORT    UART_NUM_0
#define CMD_UART_RX_BUF  512

static int io_init(void) {
    printf("[CLI] Backend: UART0\r\n");
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(CMD_UART_PORT, CMD_UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CMD_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CMD_UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return 0;
}
static int io_read_byte(uint8_t *c) {
    int n = uart_read_bytes(CMD_UART_PORT, c, 1, pdMS_TO_TICKS(10));
    return (n == 1) ? 1 : 0;
}
static void io_write_str(const char *s) {
    uart_write_bytes(CMD_UART_PORT, s, strlen(s));
}
static void io_putc(int c) {
    char b = (char)c;
    uart_write_bytes(CMD_UART_PORT, &b, 1);
}
#endif

// ====== Command task ======
static void command_task(void *arg)
{
    #define CMD_HISTORY_SIZE 20
    #define CMD_MAX_LEN 256

    static char history[CMD_HISTORY_SIZE][CMD_MAX_LEN];
    static int history_count = 0;
    static int history_index = -1;

    static void add_history(const char *cmd) {
        if (strlen(cmd) == 0) return;
        if (history_count < CMD_HISTORY_SIZE) {
            strcpy(history[history_count++], cmd);
        } else {
            // posuň históriu dole, najnovší ide na koniec
            memmove(history, history + 1, sizeof(history) - CMD_MAX_LEN);
            strcpy(history[CMD_HISTORY_SIZE - 1], cmd);
        }
    }

    // Initialize backend before any read/write
    if (io_init() != 0) {
    #if KROSCHU_BACKEND_USB
        ESP_LOGE(TAG, "USB Serial/JTAG driver install failed – set console to USB in menuconfig.");
    #else
        ESP_LOGE(TAG, "UART initialization failed.");
    #endif
        vTaskDelete(NULL);
        return;
    }

    io_write_str(">>> CLI ready. Type 'commands' to list available commands.\r\n> ");

    uint8_t c, prev = 0;
    char line[256];
    int idx = 0;

    for (;;) {
        if (io_read_byte(&c)) {
            // normalize CRLF: swallow '\n' if previous was '\r'
            if ((c == '\n') && (prev == '\r')) { prev = 0; continue; }

            if (c == '\r' || c == '\n') {           // Enter
                line[idx] = '\0';
                if (idx > 0) {
                    command_parse_and_execute(line);
                    idx = 0;
                }
                io_write_str("\r\n> ");
                prev = c;
                continue;
            }
            if ((c == 0x08 || c == 0x7F) && idx > 0) { // Backspace
                idx--;
                io_write_str("\b \b");
                prev = c;
                continue;
            }
            if (idx < (int)sizeof(line) - 1) {
                line[idx++] = (char)c;
                io_putc((int)c); // echo
            }
            prev = c;
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

void kroschuthread_commands_start(void)
{
    xTaskCreate(command_task, "kroschuthread_cmd", 4096, NULL, 3, NULL);
}
