idf_component_register(SRCS "sdcard.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver fatfs sdmmc esp_driver_sdspi log freertos esp_timer)
