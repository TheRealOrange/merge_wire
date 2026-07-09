idf_component_register(SRCS "src/merge_wire.c" "src/driver_common.c" "src/driver_rs485.c" "src/driver_fullduplex.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES esp_driver_uart esp_ringbuf esp_timer esp_hal_uart esp_hw_support)
