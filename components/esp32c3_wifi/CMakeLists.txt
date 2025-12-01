idf_component_register(SRCS "esp32c3_wifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_wifi esp_netif esp_event esp_http_server esp_timer nvs_flash esp-tls esp_driver_gpio)


# esp_wifi nvs_flash esp-tls esp_netif esp_http_server esp_driver_gpio esp_timer