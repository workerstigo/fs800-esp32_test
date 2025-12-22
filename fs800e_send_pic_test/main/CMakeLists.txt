idf_component_register(SRCS "main.c" "app_httpd.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_http_server nvs_flash esp_wifi esp_event driver esp32-camera esp_timer esp_driver_uart esp_driver_gpio esp_driver_ledc vfs mbedtls)
