#include "kroschuthread_commands.h"
#include "kroschuthread_protocol.h"
#include "kroschuthread_nodeid.h" // Tento súbor nám poskytuje nodeid_get()

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
static void cmd_whoami(int argc, char **argv); // Deklarácia pre whoami

// ====== Command table ======
static const command_t g_commands[] = {
    { "sendto",   "sendto <node_id_hex> <message>  - Send a text message to target node", cmd_sendto },
    { "commands", "commands                         - List all available commands",       cmd_commands },
    { "nodes",    "nodes                            - Print known node table",            cmd_nodes },
    { "whoami",   "whoami                           - Print local device info",           cmd_whoami }, // Príkaz whoami
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
        printf("Usage: sendto <hex_addr> <message>\n");
        return;
    }
    uint16_t dest = (uint16_t)strtol(argv[1], NULL, 16);

    char msgbuf[KROSCHUTHREAD_MAX_PAYLOAD_SIZE + 1];
    size_t mlen = join_args(msgbuf, sizeof(msgbuf), argc, argv, 2);

    if (mlen == 0) {
        printf("Message is empty\n");
        return;
    }

    printf("Sending to node 0x%04X: '%s'\n", dest, msgbuf);

    kroschuthread_status_t st = kroschuthread_protocol_send_data_no_ack(
        (const uint8_t*)msgbuf, mlen, dest, KROSCHUTHREAD_DATA_PORT);

    if (st == KROSCHUTHREAD_STATUS_SUCCESS) {
        printf("Message sent\n");
    } else {
        printf("Error: Send failed (%d)\n", st);
    }
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

    esp_log_level_t original_level = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_INFO);

    printf("\n--- Node Table ---\n");
    node_table_debug_print(); 
    printf("------------------\n\n");

    esp_log_level_set("*", original_level);
}

// ==========================================================
// !!! TU JE UPRAVENÁ FUNKCIA !!!
// ==========================================================
/**
 * @brief Prints local device information
 */
static void cmd_whoami(int argc, char **argv)
{
    (void)argc; (void)argv;

    // !!! UPRAVENÉ !!!
    // Používame funkciu nodeid_get(), ktorá existuje v kroschuthread_nodeid.c
    //
    uint16_t my_addr = nodeid_get();

    // Výpis začína novým riadkom
    printf("\n=== Local Device Info ===\n");
    printf("Node ID:      0x%04X\n", my_addr);
    printf("Channel:      %d\n", KROSCHUTHREAD_CHANNEL);
    printf("Data Port:    %d\n", KROSCHUTHREAD_DATA_PORT);
    printf("ACK Port:     %d\n", KROSCHUTHREAD_ACK_PORT);
    printf("Max Payload:  %d bytes\n", KROSCHUTHREAD_MAX_PAYLOAD_SIZE);
    printf("===========================\n");
    
    // Výpis končí prázdnym riadkom
    printf("\n"); 
}
// ==========================================================
// !!! KONIEC UPRAVENEJ FUNKCIE !!!
// ==========================================================


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
    printf("Unknown command: %s\n", argv[0]);
}

// ====== Backend I/O primitives ======
#if KROSCHU_BACKEND_USB
// --- USB Serial/JTAG (needs driver_install before first read/write) ---
static int s_usb_ready = 0;

static int io_init(void) {
    printf("[CLI] Backend: USB Serial/JTAG\r\n");

    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        s_usb_ready = 1;
        return 0;
    }
    printf("Error: usb_serial_jtag_driver_install failed: %s\n", esp_err_to_name(err));
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

// --- BEGIN HISTORY/LINE EDITING ---

#define CMD_HISTORY_SIZE 20
#define CMD_MAX_LEN 256

static char g_history[CMD_HISTORY_SIZE][CMD_MAX_LEN];
static int g_history_count = 0;
static int g_history_nav_index = 0; // Current navigation position in history
static const char s_empty_line[1] = {0};

static void add_history(const char *cmd)
{
    if (strlen(cmd) == 0) return;
    if (g_history_count > 0) {
        int last_idx = (g_history_count == CMD_HISTORY_SIZE) ? (CMD_HISTORY_SIZE - 1) : (g_history_count - 1);
        if (strcmp(g_history[last_idx], cmd) == 0) {
            g_history_nav_index = g_history_count;
            return;
        }
    }
    if (g_history_count < CMD_HISTORY_SIZE) {
        strcpy(g_history[g_history_count], cmd);
        g_history_count++;
    } else {
        memmove(g_history, g_history + 1, (CMD_HISTORY_SIZE - 1) * CMD_MAX_LEN);
        strcpy(g_history[CMD_HISTORY_SIZE - 1], cmd);
    }
    g_history_nav_index = g_history_count;
}

static void set_console_line(const char *new_line, char *line_buf, int *line_idx, int line_buf_size)
{
    io_write_str("\r> ");
    for (int i = 0; i < *line_idx; i++) {
        io_putc(' ');
    }
    io_write_str("\r> ");

    strncpy(line_buf, new_line, line_buf_size - 1);
    line_buf[line_buf_size - 1] = '\0';
    *line_idx = strlen(line_buf);

    io_write_str(line_buf);
}

// --- END HISTORY/LINE EDITING ---


static void command_task(void *arg)
{
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
    char line[CMD_MAX_LEN];
    int idx = 0;

    typedef enum { STATE_NORMAL, STATE_ESC, STATE_ESC_BRACKET } esc_state_t;
    static esc_state_t s_state = STATE_NORMAL;

    g_history_nav_index = 0;

    for (;;) {
        if (io_read_byte(&c)) {
            if ((c == '\n') && (prev == '\r')) { prev = 0; continue; }

            if (s_state == STATE_NORMAL) {
                if (c == 0x1B) {
                    s_state = STATE_ESC;
                }
                else if (c == '\r' || c == '\n') {
                    line[idx] = '\0';
                    io_write_str("\r\n");
                    if (idx > 0) {
                        add_history(line);
                        command_parse_and_execute(line);
                        idx = 0;
                    }
                    io_write_str("> ");
                }
                else if ((c == 0x08 || c == 0x7F) && idx > 0) {
                    idx--;
                    io_write_str("\b \b");
                }
                else if (c >= 0x20 && c <= 0x7E && idx < (int)sizeof(line) - 1) {
                    line[idx++] = (char)c;
                    io_putc((int)c);
                }
            }
            else if (s_state == STATE_ESC) {
                s_state = (c == '[') ? STATE_ESC_BRACKET : STATE_NORMAL;
            }
            else if (s_state == STATE_ESC_BRACKET) {
                s_state = STATE_NORMAL;
                if (c == 'A') { // Up
                    if (g_history_nav_index > 0) {
                        g_history_nav_index--;
                        set_console_line(g_history[g_history_nav_index], line, &idx, sizeof(line));
                    }
                } else if (c == 'B') { // Down
                    if (g_history_nav_index < g_history_count) {
                        g_history_nav_index++;
                        if (g_history_nav_index == g_history_count) {
                            set_console_line(s_empty_line, line, &idx, sizeof(line));
                        } else {
                            set_console_line(g_history[g_history_nav_index], line, &idx, sizeof(line));
                        }
                    }
                }
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