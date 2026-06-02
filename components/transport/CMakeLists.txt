idf_component_register(
    SRCS "ntrip_transport.c" "lora_transport.c"
    INCLUDE_DIRS "include"
    REQUIRES ntrip lora freertos esp_ringbuf log driver
)
