idf_component_register(
    SRCS "imu_spi_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_common
        log
)
