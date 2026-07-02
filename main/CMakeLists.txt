idf_component_register(SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES gnss ntrip imu display encoder ui ups sdcard transport config
                                 base_station ble_uart
                                 esp_wifi esp_event nvs_flash esp_netif driver)
