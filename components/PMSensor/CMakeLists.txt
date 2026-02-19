idf_component_register(
    SRCS "drivers/PMS5003Base.cpp"
         "drivers/PMS5003.cpp"
         "drivers/PMS5003T.cpp"
         "drivers/SPS30Sensor.cpp"
    INCLUDE_DIRS "include" "drivers"
    REQUIRES airgradient-common AirgradientSerial sps30
)
