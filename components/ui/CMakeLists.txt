idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "include"
    REQUIRES display encoder gnss imu config freertos esp_system
)
