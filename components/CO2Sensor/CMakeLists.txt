idf_component_register(
    SRCS "drivers/modbus_crc.cpp"
         "drivers/S8.cpp"
         "drivers/Sunlight.cpp"
         "drivers/STCC4Sensor.cpp"
    INCLUDE_DIRS "include" "drivers"
    REQUIRES airgradient-common AirgradientSerial stcc4
)
