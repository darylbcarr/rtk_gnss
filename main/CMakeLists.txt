idf_component_register(SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES gnss ntrip imu display encoder ui transport config
                                 base_station
                                 esp_wifi esp_event nvs_flash esp_netif driver)
