idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "include"
    REQUIRES display encoder gnss imu ups base_station config freertos esp_system
)
