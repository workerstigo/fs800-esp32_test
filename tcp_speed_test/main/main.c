#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h" // 用於計時
#include <string.h>
#include <stdlib.h>

#define RX 16
#define TX 17
#define TEST_FILE_SIZE (50 * 1024) // 模擬 50KB 圖片
#define CHUNK_SIZE 1024

static const char *TAG = "TEST_APP";

// 傳輸測試函式
void run_speed_test() {
    uint8_t *dummy_data = (uint8_t *)malloc(CHUNK_SIZE);
    // 填入一些假資料 (A, B, C...)
    for (int i = 0; i < CHUNK_SIZE; i++) dummy_data[i] = 'A' + (i % 26);

    ESP_LOGI(TAG, "Starting Speed Test: Sending %d bytes to 4G Modular...", TEST_FILE_SIZE);
    
    int64_t start_time = esp_timer_get_time();
    int total_sent = 0;

    while (total_sent < TEST_FILE_SIZE) {
        int to_send = CHUNK_SIZE;
        if (total_sent + to_send > TEST_FILE_SIZE) {
            to_send = TEST_FILE_SIZE - total_sent;
        }

        // 發送資料到 4G 模組 (UART2)
        uart_write_bytes(UART_NUM_2, (const char *)dummy_data, to_send);
        total_sent += to_send;
        
        // 可選：每傳送一點點印個小點，避免 Console 刷太快，或什麼都不做保持最高速
        // uart_write_bytes(UART_NUM_0, ".", 1); 
    }
    
    int64_t end_time = esp_timer_get_time();
    free(dummy_data);

    float duration_sec = (end_time - start_time) / 1000000.0;
    float speed_kbps = (TEST_FILE_SIZE / 1024.0) / duration_sec;

    char report[128];
    int len = snprintf(report, sizeof(report), "\nDone! Time: %.3fs, Speed: %.2f KB/s\n", duration_sec, speed_kbps);
    uart_write_bytes(UART_NUM_0, report, len);
}

void app_main(void) {
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Serial (UART0) Console
    uart_driver_install(UART_NUM_0, CHUNK_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &config);

    // Serial2 (UART2) 4G Module
    uart_driver_install(UART_NUM_2, CHUNK_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &config);
    uart_set_pin(UART_NUM_2, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 提示訊息
    const char *msg = "\n=== UART 4G Bridge ===\nType 't' to start speed test (send 50KB).\nOther keys are transparently bridged.\n";
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));

    uint8_t data[256]; // 臨時小緩衝區
    while (1) {
        // --- 1. 檢查 Console 輸入 ---
        int len = uart_read_bytes(UART_NUM_0, data, sizeof(data), 0);
        if (len > 0) {
            // 檢查是否觸發測試 (只有收到單純 't' 時觸發，避免影響正常的 AT+T 指令等)
            // 這裡簡單判定：如果包含 't' 就觸發 (可依需求改更嚴謹)
            bool trigger = false;
            for(int i=0; i<len; i++) {
                if(data[i] == 't') { trigger = true; break; }
            }

            if (trigger) {
                run_speed_test();
            } else {
                // 否則當作普通指令轉傳給 4G
                uart_write_bytes(UART_NUM_2, (const char*)data, len);
            }
        }

        // --- 2. 檢查 4G 回傳 ---
        len = uart_read_bytes(UART_NUM_2, data, sizeof(data), 0);
        if (len > 0) {
            uart_write_bytes(UART_NUM_0, (const char*)data, len);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // 讓出 CPU
    }
}