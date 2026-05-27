idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        shift_register
        board_detect
        hmc100x
        ad7779
        fatfs
        sdmmc
        esp_driver_gpio
        esp_driver_sdmmc
        esp_driver_sdspi
        esp_driver_spi
        esp_driver_uart
)
