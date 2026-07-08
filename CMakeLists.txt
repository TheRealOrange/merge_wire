idf_component_register(SRCS "src/merge_wire.c" "src/driver.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES esp_driver_uart esp_ringbuf)
