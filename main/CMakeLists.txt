idf_component_register(
    SRCS
        "go.cpp"
        "button_service.cpp"
        "gps_service.cpp"
        "nand_storage_service.cpp"
        "ui/u8g2_canvas.cpp"
        "ui/dashboard_ui.cpp"
    INCLUDE_DIRS "."
    REQUIRES edp-ssd1680x driver u8g2 sps30 freertos libnmea spi_nand_flash WiFiManager esp_http_client cap1203 esp_event esp_timer console esp_driver_usb_serial_jtag bq25629
)
