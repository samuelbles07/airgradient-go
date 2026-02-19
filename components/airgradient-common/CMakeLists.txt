idf_component_register(
    SRCS "RTOS.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_timer
)
