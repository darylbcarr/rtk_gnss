idf_component_register(SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES gnss ntrip imu display encoder ui
                                 esp_wifi esp_event nvs_flash esp_netif driver)
