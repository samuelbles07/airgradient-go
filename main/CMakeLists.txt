idf_component_register(
    SRCS
        "airgradient-go.cpp"
        "gps_service.cpp"
        "nand_storage_service.cpp"
        "ui/u8g2_canvas.cpp"
        "ui/dashboard_ui.cpp"
    INCLUDE_DIRS "."
    REQUIRES edp-ssd1680x driver u8g2 sps30 freertos libnmea spi_nand_flash
)
