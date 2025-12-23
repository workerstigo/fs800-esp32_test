idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash driver esp32-camera esp_timer esp_driver_uart esp_driver_gpio esp_driver_ledc vfs mbedtls)
