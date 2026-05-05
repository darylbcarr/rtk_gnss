idf_component_register(SRCS "ntrip.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_netif log freertos)
