#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
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

#define ESP_WIFI_SSID      ""
#define ESP_WIFI_PASS      ""

#define RX_PIN 13
#define TX_PIN 14 
#define UART_MOD_NUM UART_NUM_2
#define BUF_SIZE 2048

static bool g_4g_connected = false;

// --- AT Commands for 4G Module ---
const char* atInitCommands[] = {
    "AT",
    "+++", 
    "AT",
    "AT+E=OFF",
    "AT+APN=internet,,,0",
    "AT+SOCKEN=ON",
    "AT+SOCK=TCP,1.1.1.1,5000",
    "AT+SOCKSL=LONG",
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
        // TURBO MODE: 500ms stabilization delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    free(resp_buf);
    return g_4g_connected;
}

// --- Image to Text Transmission Task ---
void image_transmission_task(void *pvParameters)
{
    uint8_t *io_buf = (uint8_t *) heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!io_buf) io_buf = (uint8_t *) malloc(BUF_SIZE);

    int64_t lastSendTime = 0;
    int64_t lastHeartbeat = 0;
    const int64_t captureIntervalMs = 1000; // TURBO MODE: 1 FPS
    const int64_t heartbeatIntervalMs = 5000;

    init_4g_module();

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        if (now - lastHeartbeat > heartbeatIntervalMs) {
            ESP_LOGI(TAG, "Heartbeat. Status: %s. Use a strong 5V power supply!", 
                     g_4g_connected ? "CONNECTED" : "DISCONNECTED");
            lastHeartbeat = now;
        }

        if (!g_4g_connected) {
            ESP_LOGW(TAG, "Not connected. Re-initializing...");
            init_4g_module();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (now - lastSendTime > captureIntervalMs) {
            lastSendTime = now;
            ESP_LOGI(TAG, "Capturing Image...");

            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                size_t out_len = 0;
                mbedtls_base64_encode(NULL, 0, &out_len, fb->buf, fb->len);
                
                unsigned char *base64_buffer = (unsigned char *)heap_caps_malloc(out_len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (base64_buffer) {
                    if (mbedtls_base64_encode(base64_buffer, out_len, &out_len, fb->buf, fb->len) == 0) {
                        base64_buffer[out_len] = '\0';
                        ESP_LOGI(TAG, "Sending Image (%d bytes)...", (int)out_len);

                        // TURBO MODE: Rapid flush
                        uart_flush_input(UART_MOD_NUM);
                        vTaskDelay(pdMS_TO_TICKS(50)); 

                        uart_write_bytes(UART_MOD_NUM, "IMAGE_START\n", 12);
                        
                        size_t sent = 0;
                        bool fail = false;
                        while(sent < out_len) {
                            size_t to_send = (out_len - sent > 2048) ? 2048 : (out_len - sent); // Larger chunks
                            
                            int rx_len = uart_read_bytes(UART_MOD_NUM, io_buf, BUF_SIZE, 0);
                            if (rx_len > 0) {
                                if (strstr((char*)io_buf, "^boot.rom") || strstr((char*)io_buf, "CLOSED") || strstr((char*)io_buf, "ERROR")) {
                                    ESP_LOGE(TAG, "TURBO FAIL - Power/Connection Lost!");
                                    fail = true;
                                    break;
                                }
                            }

                            int written = uart_write_bytes(UART_MOD_NUM, (const char *)(base64_buffer + sent), to_send);
                            if (written < 0) {
                                fail = true;
                                break;
                            }
                            sent += written;
                            if (sent % 4096 < 2048) ESP_LOGI(TAG, "TX: %d/%d (TURBO)", (int)sent, (int)out_len);
                            
                            // TURBO pacing: 10ms
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        
                        if (!fail) {
                            uart_write_bytes(UART_MOD_NUM, "IMAGE_END\n", 10);
                            ESP_LOGI(TAG, "Transmission Finished.");
                        } else {
                            g_4g_connected = false;
                        }
                    }
                    free(base64_buffer);
                }
                esp_camera_fb_return(fb);
            }
        }

        int len = uart_read_bytes(UART_MOD_NUM, io_buf, BUF_SIZE, pdMS_TO_TICKS(50));
        if (len > 0) {
            ESP_LOGI(TAG, "4G RX: %.*s", len, (char*)io_buf);
            if (strstr((char*)io_buf, "^boot.rom")) {
                 ESP_LOGE(TAG, "DETECTED MODULE REBOOT (Power issue?)");
                 g_4g_connected = false;
            } else if (strstr((char*)io_buf, "CLOSED") || strstr((char*)io_buf, "ERROR")) {
                g_4g_connected = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (io_buf) free(io_buf);
    vTaskDelete(NULL);
}

bool psramFound() {
#ifdef CONFIG_SPIRAM_SUPPORT
    return true; 
#else
    return false;
#endif
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .sta = { .ssid = ESP_WIFI_SSID, .password = ESP_WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
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

    wifi_init_sta();

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 5000000; // Stabilize DMA
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 35; // Turbo Compression
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
        sensor_t * s = esp_camera_sensor_get();
        if (s->id.PID == OV5640_PID) s->set_vflip(s, 1);
    }
    
    // Prototypes for app_httpd
    extern void startCameraServer();
    extern void setupLedFlash();
    startCameraServer();

    // Use priority 1 to ensure Camera DMA interrupts and WiFi have highest precedence
    xTaskCreate(image_transmission_task, "img_to_4g", 16384, NULL, 1, NULL);
}
