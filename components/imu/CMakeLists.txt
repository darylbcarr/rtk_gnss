idf_component_register(SRCS "imu.c" "imu_cal.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver log esp_timer)
