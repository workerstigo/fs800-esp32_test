#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "mbedtls/base64.h"
#include "esp_heap_caps.h"

// ===========================
// Configuration Info
// ===========================
#include "board_config.h"

static const char *TAG = "image_to_4g";

#define RX_PIN 13
#define TX_PIN 14 
#define UART_MOD_NUM UART_NUM_2
#define BUF_SIZE 2048

static bool g_4g_connected = false;
static int64_t g_last_server_ok_time = 0;
#define SERVER_OK_TIMEOUT_MS 10000 // 10 seconds

// --- AT Commands for 4G Module ---
const char* atInitCommands[] = {
    "AT",
    "+++", 
    "AT",
    "AT+SOCKEN=ON",
    "AT+SOCK=TCP,1.1.1.1,5000",
    "AT+S"
};
const int initCmdCount = sizeof(atInitCommands) / sizeof(atInitCommands[0]);

// Helper to flush UART RX
void flush_uart_rx() {
    uint8_t dummy[128];
    while (uart_read_bytes(UART_MOD_NUM, dummy, sizeof(dummy), 0) > 0);
}

// Function to (re)initialize 4G module
bool init_4g_module() {
    ESP_LOGI(TAG, "Initializing 4G Module (Wait for Connection)...");
    uint8_t *resp_buf = (uint8_t *) malloc(BUF_SIZE);
    
    g_4g_connected = false;
    for (int i = 0; i < initCmdCount; i++) {
        ESP_LOGI(TAG, "CMD: %s", atInitCommands[i]);
        if (strcmp(atInitCommands[i], "+++") == 0) {
            uart_write_bytes(UART_MOD_NUM, "+++", 3);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            flush_uart_rx();
            uart_write_bytes(UART_MOD_NUM, atInitCommands[i], strlen(atInitCommands[i]));
            uart_write_bytes(UART_MOD_NUM, "\r\n", 2);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        int len = uart_read_bytes(UART_MOD_NUM, resp_buf, BUF_SIZE, pdMS_TO_TICKS(200));
        if (len > 0) {
            ESP_LOGI(TAG, "Resp: %.*s", len, (char*)resp_buf);
        }
    }
    
    // Wait for "FS@TCP CONNECTED" for up to 10 seconds
    ESP_LOGI(TAG, "Waiting for TCP Connection Verification...");
    int retry = 0;
    while (retry++ < 50) { // 50 * 200ms = 10s
        int len = uart_read_bytes(UART_MOD_NUM, resp_buf, BUF_SIZE, pdMS_TO_TICKS(200));
        if (len > 0) {
            ESP_LOGI(TAG, "4G Status: %.*s", len, (char*)resp_buf);
            if (strstr((char*)resp_buf, "CONNECTED")) {
                ESP_LOGI(TAG, "4G Connection Verified (Ready for Data)");
                g_4g_connected = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!g_4g_connected) {
        ESP_LOGW(TAG, "4G Connection Timeout. Module might still be connecting...");
    } else {
        // Reset heartbeat timer on successful init
        g_last_server_ok_time = esp_timer_get_time() / 1000;
        // Wait 5 seconds for stabilization before sending data
        ESP_LOGI(TAG, "Waiting 5s for stabilization...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    free(resp_buf);
    return g_4g_connected;
}

// --- Simple Image Transmission Task ---
void image_transmission_task(void *pvParameters)
{
    uint8_t *io_buf = (uint8_t *) heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!io_buf) io_buf = (uint8_t *) malloc(BUF_SIZE);

    init_4g_module();

    while (1) {
        if (!g_4g_connected) {
            ESP_LOGW(TAG, "Not connected. Re-initializing...");
            init_4g_module();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Capturing Image...");
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            // --- STEP 1: Copy to private buffer & release FB ---
            size_t jpeg_len = fb->len;
            uint8_t *raw_buf = (uint8_t *)heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (raw_buf) {
                memcpy(raw_buf, fb->buf, jpeg_len);
                esp_camera_fb_return(fb); 
                fb = NULL;

                // --- STEP 2: Base64 Encode ---
                size_t out_len = 0;
                mbedtls_base64_encode(NULL, 0, &out_len, raw_buf, jpeg_len);
                unsigned char *base64_buffer = (unsigned char *)heap_caps_malloc(out_len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                
                if (base64_buffer) {
                    if (mbedtls_base64_encode(base64_buffer, out_len, &out_len, raw_buf, jpeg_len) == 0) {
                        base64_buffer[out_len] = '\0';
                        ESP_LOGI(TAG, "Sending Image Text (%d bytes)...", (int)out_len);

                        uart_write_bytes(UART_MOD_NUM, "IMAGE_START\n", 12);
                        size_t sent = 0;
                        while(sent < out_len) {
                            size_t to_send = (out_len - sent > 512) ? 512 : (out_len - sent);
                            uart_write_bytes(UART_MOD_NUM, (const char *)(base64_buffer + sent), to_send);
                            sent += to_send;
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        uart_write_bytes(UART_MOD_NUM, "IMAGE_END\n", 10);
                        ESP_LOGI(TAG, "Finished.");
                    }
                    free(base64_buffer);
                }
                free(raw_buf);
            } else {
                esp_camera_fb_return(fb);
            }
        }

        // Check for Server Messages & Wait
        int64_t start_wait = esp_timer_get_time() / 1000;
        const int wait_ms = 500; // Total wait time
        
        while ((esp_timer_get_time() / 1000) - start_wait < wait_ms) {
            int len = uart_read_bytes(UART_MOD_NUM, io_buf, BUF_SIZE, pdMS_TO_TICKS(100));
            if (len > 0) {
                ESP_LOGI(TAG, "Server Message: %.*s", len, (char*)io_buf);
                if (strstr((char*)io_buf, "OK")) {
                    ESP_LOGI(TAG, ">>> RECEIVED SERVER HEARTBEAT <<<");
                    g_last_server_ok_time = esp_timer_get_time() / 1000;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // --- STEP 4: Check for Server Timeout ---
        int64_t now = esp_timer_get_time() / 1000;
        if (now - g_last_server_ok_time > SERVER_OK_TIMEOUT_MS) {
            ESP_LOGE(TAG, "SERVER HEARTBEAT TIMEOUT (%d s)! Reconnecting...", (int)(SERVER_OK_TIMEOUT_MS/1000));
            g_4g_connected = false;
        }
    }
} 

bool psramFound() {
#ifdef CONFIG_SPIRAM_SUPPORT
    return true; 
#else
    return false;
#endif
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_MOD_NUM, BUF_SIZE * 4, BUF_SIZE * 4, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_MOD_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_MOD_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 5000000;
    config.frame_size = FRAMESIZE_QVGA; 
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST; 
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 35; 
    config.fb_count = 2;

    ESP_ERROR_CHECK(esp_camera_init(&config));
    
    // Simple priority 1 task
    xTaskCreate(image_transmission_task, "img_tx", 16384, NULL, 1, NULL);
}
