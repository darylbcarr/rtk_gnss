idf_component_register(
    SRCS         "ble_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES     bt log
)

# NimBLE headers are spread across many subdirectories inside bt and are
# not propagated as public includes. Add every relevant subtree explicitly.
idf_component_get_property(bt_dir bt COMPONENT_DIR)
set(_nimble "${bt_dir}/host/nimble/nimble")
target_include_directories(${COMPONENT_LIB} PRIVATE
    # NimBLE porting / NPL layer
    "${_nimble}/porting/nimble/include"
    "${_nimble}/porting/npl/freertos/include"
    # NimBLE core & host
    "${_nimble}/nimble/include"
    "${_nimble}/nimble/host/include"
    "${_nimble}/nimble/host/util/include"
    # Transport layer (nimble/transport.h)
    "${_nimble}/nimble/transport/include"
    # Host services
    "${_nimble}/nimble/host/services/gap/include"
    "${_nimble}/nimble/host/services/gatt/include"
    # ESP-IDF NimBLE port / HCI
    "${bt_dir}/host/nimble/port/include"
    "${bt_dir}/host/nimble/esp-hci/include"
    # bt-level porting (esp_nimble_hci, npl shim)
    "${bt_dir}/porting/npl/freertos/include"
    "${bt_dir}/porting/include"
    "${bt_dir}/porting/transport/include"
    # bt public include (ESP32-S3 uses the root include dir)
    "${bt_dir}/include"
)
