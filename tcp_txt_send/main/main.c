#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define RX 16
#define TX 17

void app_main(void) {
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Serial (UART0)
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &config);

    // Serial2 (UART2)
    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &config);
    uart_set_pin(UART_NUM_2, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t data;
    while (1) {
        // Serial -> 4G
        // 使用 while 確保讀取緩衝區所有資料 (避免 vTaskDelay 造成溢出)
        while (uart_read_bytes(UART_NUM_0, &data, 1, 0) > 0) {
            uart_write_bytes(UART_NUM_2, (const char*)&data, 1);
        }

        // 4G -> Serial
        while (uart_read_bytes(UART_NUM_2, &data, 1, 0) > 0) {
            uart_write_bytes(UART_NUM_0, (const char*)&data, 1);
        }

        vTaskDelay(1); // 讓出 CPU 避免 Watchdog 觸發
    }
}
