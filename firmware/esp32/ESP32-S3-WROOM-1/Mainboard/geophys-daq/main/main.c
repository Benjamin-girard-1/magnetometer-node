/**
 * @file main.c
 * @brief Side-by-side smoke test for the LSM6DSV and SCL3300 drivers.
 *
 * Order matters: we initialize the SCL3300 FIRST. This is not arbitrary.
 *
 * Both devices share the SPI bus. Each driver's port_init() configures its
 * own CS pin via ESP-IDF's spics_io_num, which drives the line high. But
 * BEFORE that init runs, the GPIO is in its boot default — floating, weak
 * internal pulls, undefined logic level. If we initialize LSM6DSV first
 * and start sending it WHO_AM_I, the SCL3300's CSB is still floating, and
 * its MISO output can stomp on the LSM6DSV's response, producing things
 * like 0x30 instead of 0x70 (single-bit corruption from bus contention).
 *
 * Initializing SCL3300 first parks GPIO46 high as a side effect of its
 * port_init(), and the SCL3300 then ignores everything on the bus until
 * it sees its own CS go low.
 */

#include "lsm6dsv.h"
#include "scl3300.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "app";

void app_main(void)
{
    /* Power-on settle. SCL3300 mode 1 needs >= 25 ms after VDD is up; we   */
    /* give 150 ms to be comfortable across both parts.                     */
    vTaskDelay(pdMS_TO_TICKS(150));

    /* --- SCL3300 FIRST --------------------------------------------------- */
    /* Initializing this driver first drives GPIO46 high (CS_SCL) before    */
    /* any LSM6DSV traffic happens, eliminating the bus-contention window.  */
    scl3300_t inc = {0};
    {
        scl3300_config_t cfg = SCL3300_CONFIG_DEFAULT();
        scl3300_status_t st = scl3300_init(&inc, &cfg);
        if (st != SCL3300_OK) {
            ESP_LOGE(TAG, "scl3300_init failed: %d (WHOAMI=0x%02X)",
                     st, inc.last_whoami);
            return;
        }
        ESP_LOGI(TAG, "SCL3300 OK (WHOAMI=0x%02X)", inc.last_whoami);
    }

    /* --- LSM6DSV --------------------------------------------------------- */
    lsm6dsv_t imu = {0};
    {
        lsm6dsv_config_t cfg = LSM6DSV_CONFIG_DEFAULT();
        lsm6dsv_status_t st = lsm6dsv_init(&imu, &cfg);
        if (st != LSM6DSV_OK) {
            ESP_LOGE(TAG, "lsm6dsv_init failed: %d (WHO_AM_I=0x%02X)",
                     st, imu.last_who_am_i);
            return;
        }
        ESP_LOGI(TAG, "LSM6DSV OK (WHO_AM_I=0x%02X)", imu.last_who_am_i);
    }

    /* --- Loop ------------------------------------------------------------ */
    while (1) {
        lsm6dsv_sample_t a;
        scl3300_sample_t b;

        bool have_a = (lsm6dsv_read_sample(&imu, &a) == LSM6DSV_OK);
        bool have_b = (scl3300_read_sample(&inc, &b) == SCL3300_OK);

        if (have_a && have_b) {
            ESP_LOGI(TAG,
                     "LSM6 %+6.3f %+6.3f %+6.3f g  |  "
                     "SCL  %+6.3f %+6.3f %+6.3f g  "
                     "ang(%+6.2f %+6.2f %+6.2f)°  T=%5.2f/%5.2f C",
                     a.xl_x_g, a.xl_y_g, a.xl_z_g,
                     b.acc_x_g, b.acc_y_g, b.acc_z_g,
                     b.ang_x_deg, b.ang_y_deg, b.ang_z_deg,
                     a.temp_c, b.temp_c);
        } else if (!have_a) {
            ESP_LOGW(TAG, "LSM6DSV read failed");
        } else {
            ESP_LOGW(TAG, "SCL3300 read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}