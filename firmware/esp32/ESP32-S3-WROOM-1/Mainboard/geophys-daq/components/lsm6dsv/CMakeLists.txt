# LSM6DSV component for ESP-IDF.
#
# When migrating to STM32, you don't use this file at all — instead you add
# lsm6dsv.c and lsm6dsv_port_stm32.c to your STM32CubeIDE project's source
# list and add include/ + port/ to the include paths.

idf_component_register(
    SRCS
        "lsm6dsv.c"
        "port/lsm6dsv_port_esp32.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "."
        "port"
    REQUIRES
        driver        # spi_master, gpio
        esp_common
        freertos
        log
)
