#ifndef IMU_SPI_BUS_H
#define IMU_SPI_BUS_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_SPI_HOST       SPI3_HOST
#define IMU_PIN_MOSI       8
#define IMU_PIN_MISO       9
#define IMU_PIN_SCLK       3
#define LSM6DSV_PIN_CS     GPIO_NUM_17
#define SCL3300_PIN_CS     GPIO_NUM_46

esp_err_t imu_spi_bus_init_once(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SPI_BUS_H */
