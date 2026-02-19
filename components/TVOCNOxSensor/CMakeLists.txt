idf_component_register(
    SRCS "drivers/SGP41.cpp"
    INCLUDE_DIRS "include" "drivers"
    REQUIRES airgradient-common esp_driver_i2c
)
