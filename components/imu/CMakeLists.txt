idf_component_register(SRCS "imu.c" "imu_cal.c" "imu_mag.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver log esp_timer)
