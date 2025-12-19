#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define RX_PIN 16
#define TX_PIN 17

#define UART_MOD_NUM UART_NUM_2
#define UART_CON_NUM UART_NUM_0
#define BUF_SIZE 2048

const char* atCommands[] = {
    "AT",
    "AT",
    "ATI",
    "AT+CPIN?",
    "AT+ICCID",
    "AT+CEREG?",
    "AT+CSQ",
    "AT+QICSGP=1,1,\"internet\",\"\",\"\"",
    "AT+NETOPEN",
    "AT+CIPOPEN=0,\"TCP\",\"1.1.1.1\",5000"
};

const int cmdCount = sizeof(atCommands) / sizeof(atCommands[0]);
int cmdIndex = 0;
int64_t lastSend = 0;
const int64_t sendIntervalMs = 1000;

// 關鍵狀態
bool net_ready = false;

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_CON_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_CON_NUM, &uart_config);

    uart_driver_install(UART_MOD_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_MOD_NUM, &uart_config);
    uart_set_pin(UART_MOD_NUM, TX_PIN, RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    uint8_t *data = malloc(BUF_SIZE);

    vTaskDelay(pdMS_TO_TICKS(3000));

    uart_write_bytes(UART_CON_NUM,
        "\r\n=== ESP32 AT Bridge Started ===\r\n", 36);

    lastSend = esp_timer_get_time() / 1000;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        // ===== 自動送 AT =====
        if (cmdIndex < cmdCount && (now - lastSend) > sendIntervalMs) {

            // CIPOPEN 一定要等 net_ready
            if (strstr(atCommands[cmdIndex], "CIPOPEN") && !net_ready) {
                // 不送，等狀態
            } else {
                uart_write_bytes(UART_CON_NUM, "TX: ", 4);
                uart_write_bytes(UART_CON_NUM,
                                 atCommands[cmdIndex],
                                 strlen(atCommands[cmdIndex]));
                uart_write_bytes(UART_CON_NUM, "\r\n", 2);

                uart_write_bytes(UART_MOD_NUM,
                                 atCommands[cmdIndex],
                                 strlen(atCommands[cmdIndex]));
                uart_write_bytes(UART_MOD_NUM, "\r\n", 2);

                lastSend = now;
                cmdIndex++;
            }
        }

        // ===== Module -> Console =====
        int len2 = uart_read_bytes(UART_MOD_NUM,
                                   data, BUF_SIZE,
                                   pdMS_TO_TICKS(10));
        if (len2 > 0) {
            uart_write_bytes(UART_CON_NUM, (char*)data, len2);

            // ★ 關鍵：NETOPEN 成功「或」ERROR:90 都算 ready
            if (strstr((char*)data, "+NETOPEN:SUCCESS") ||
                strstr((char*)data, "ERROR: 90")) {
                net_ready = true;
            }
        }

        // ===== Console -> Module =====
        int len0 = uart_read_bytes(UART_CON_NUM, data, BUF_SIZE, 0);
        if (len0 > 0) {
            uart_write_bytes(UART_MOD_NUM, (char*)data, len0);
        }
    }
}
