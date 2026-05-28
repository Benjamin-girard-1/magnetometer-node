# SCL3300 component for ESP-IDF.
#
# When migrating to STM32, you don't use this file at all — instead you add
# scl3300.c and scl3300_port_stm32.c to your STM32CubeIDE project's source
# list and add include/ + port/ to the include paths.

idf_component_register(
    SRCS
        "scl3300.c"
        "port/scl3300_port_esp32.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "."
        "port"
    REQUIRES
        imu_spi_bus
        driver        # spi_master, gpio
        esp_common
        esp_timer     # for the TLH gap timing
        freertos
        log
)
