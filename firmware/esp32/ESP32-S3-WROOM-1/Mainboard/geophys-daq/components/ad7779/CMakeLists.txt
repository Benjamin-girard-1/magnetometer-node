# ESP-IDF component definition.
# Place this file alongside the driver sources in your component dir, e.g.:
#   components/ad7779/CMakeLists.txt
#   components/ad7779/include/...
#   components/ad7779/src/ad7779.c
#   components/ad7779/port/ad7779_hal_esp32.c

idf_component_register(
    SRCS
        "ad7779.c"
        "ad7779_crc.c"
        "port/ad7779_hal_esp32.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_timer
        freertos
)
