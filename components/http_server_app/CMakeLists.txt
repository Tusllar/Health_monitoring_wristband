idf_component_register(SRCS "http_server_app.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_wifi esp_netif esp_event esp_http_server esp_timer max30100 WHOLE_ARCHIVE)


target_add_binary_data(${COMPONENT_TARGET} "../../main/index.html" TEXT)