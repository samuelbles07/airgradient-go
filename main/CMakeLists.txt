# Maybe don't directly include "driver" instead "esp_driver_i2c" etc
set (
    components
    edp-ssd1680x
    driver
    u8g2
    PMSensor
    TVOCNOxSensor
    dps368
    freertos
    libnmea
    spi_nand_flash
    WiFiManager
    esp_http_client
    cap1203
    esp_event
    esp_timer
    console
    esp_driver_usb_serial_jtag
    bq25629
    CO2Sensor
)

set(srcs
    "go.cpp"
    "ble_stream.cpp"
    "button_service.cpp"
    "gps_service.cpp"
    "nand_storage_service.cpp"
    "ui/u8g2_canvas.cpp"
    "ui/dashboard_ui.cpp"
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "."
    REQUIRES "${components}"
)
