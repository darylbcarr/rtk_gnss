idf_component_register(
    SRCS "base_app.c"
    INCLUDE_DIRS "include"
    REQUIRES gnss transport log freertos esp_timer
)
