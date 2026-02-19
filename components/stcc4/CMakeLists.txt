idf_component_register(
    SRCS "src/stcc4.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_common log
)
