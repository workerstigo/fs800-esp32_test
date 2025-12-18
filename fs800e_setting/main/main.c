/*
 * Arduino to ESP-IDF Port: AT Command Bridge
 *
 * Logic:
 * 1. Initialize UART0 (Console) and UART2 (Module, RX=16, TX=17).
 * 2. Send AT commands sequence every 1000ms.
 * 3. Bridge traffic between UART0 and UART2 transparently.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

// Pin Definitions
#define RX_PIN 16
#define TX_PIN 17

// UART Ports
#define UART_MOD_NUM UART_NUM_2 // Corresponds to Serial2
#define UART_CON_NUM UART_NUM_0 // Corresponds to Serial

// Buffer Size - Increased for safety
#define BUF_SIZE 2048

static const char *TAG = "AT_BRIDGE";

// Command List
const char* atCommands[] = {
    "AT",
    "+++",
    "AT+UART=115200,8,1,NONE,NONE",
    "AT+UARTFT=50",
    "AT+UARTFL=4096",
    "AT+E=ON",
    "AT+LINKDEBUG=ON",
    "AT+PARMSVER=0",
    "AT+STMSG=freestrong",
    "AT+SN=",
    "AT+RSTIM=0",
    "AT+APN=internet,,,0",
    "AT+NTPEN=ON",
    "AT+NTPSVR=cn.ntp.org.cn,cn.pool.ntp.org",
    "AT+NTPTM=60",
    "AT+WKMOD=NET",
    "AT+SOCKEN=ON",
    "AT+SOCK=TCP,1.1.1.1,5000",
    "AT+SOCKSL=LONG",
    "AT+SOCKSSL=OFF",
    "AT+SOCKRSTIM=5",
    "AT+SOCKRSNUM=60",
    "AT+KEEPALIVE=ON,60,15,3",
    "AT+HEARTEN=OFF",
    "AT+REGEN=OFF",
    "AT+S"
};

const int cmdCount = sizeof(atCommands) / sizeof(atCommands[0]);
int cmdIndex = 0;
int64_t lastSend = 0;
const int64_t sendIntervalMs = 1000;

void app_main(void)
{
    // Common UART Config
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 1. Setup UART0 (Console/Serial)
    ESP_ERROR_CHECK(uart_driver_install(UART_CON_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_CON_NUM, &uart_config));

    // 2. Setup UART2 (Module/Serial2)
    ESP_ERROR_CHECK(uart_driver_install(UART_MOD_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_MOD_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_MOD_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Safety: Enable Pullup on RX pin to avoid noise if floating
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    
    // Initial delay to let system and module power up fully
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    const char *start_msg = "\r\n=== ESP32 AT Bridge Started ===\r\n";
    uart_write_bytes(UART_CON_NUM, start_msg, strlen(start_msg));

    // Set lastSend so we wait 1s before first command
    lastSend = esp_timer_get_time() / 1000;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000; // microseconds -> milliseconds

        // --- 1. Auto Send AT Commands ---
        if (cmdIndex < cmdCount) {
             if (now - lastSend > sendIntervalMs) {
                const char *prefix = "Sending: ";
                uart_write_bytes(UART_CON_NUM, prefix, strlen(prefix));
                uart_write_bytes(UART_CON_NUM, atCommands[cmdIndex], strlen(atCommands[cmdIndex]));
                uart_write_bytes(UART_CON_NUM, "\r\n", 2);

                if (strcmp(atCommands[cmdIndex], "+++") == 0) {
                     uart_write_bytes(UART_MOD_NUM, "+++", 3);
                     // Add small delay for +++ escape usually
                     vTaskDelay(100 / portTICK_PERIOD_MS);
                } else {
                     uart_write_bytes(UART_MOD_NUM, atCommands[cmdIndex], strlen(atCommands[cmdIndex]));
                     uart_write_bytes(UART_MOD_NUM, "\r\n", 2);
                }

                lastSend = now;
                cmdIndex++;
             }
        }

        // --- 2. Module (Serial2) -> Console (Serial) ---
        // Read with small timeout (10ms)
        int len2 = uart_read_bytes(UART_MOD_NUM, data, BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (len2 > 0) {
            uart_write_bytes(UART_CON_NUM, (const char *)data, len2);
        }

        // --- 3. Console (Serial) -> Module (Serial2) ---
        // Non-blocking check
        int len0 = uart_read_bytes(UART_CON_NUM, data, BUF_SIZE, 0);
        if (len0 > 0) {
            uart_write_bytes(UART_MOD_NUM, (const char *)data, len0);
        }
    }

    free(data);
}