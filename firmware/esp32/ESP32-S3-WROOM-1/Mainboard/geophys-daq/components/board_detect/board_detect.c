/**
 * @file board_detect.c
 * @brief Expansion-card ID detection for mainboard slots.
 */

#include "board_detect.h"

#include "driver/gpio.h"

#define BOARD_SLOT_1_ID_GPIO  GPIO_NUM_39
#define BOARD_SLOT_2_ID_GPIO  GPIO_NUM_42

static gpio_num_t slot_gpio(board_slot_t slot)
{
    switch (slot) {
        case BOARD_SLOT_1: return BOARD_SLOT_1_ID_GPIO;
        case BOARD_SLOT_2: return BOARD_SLOT_2_ID_GPIO;
        default:           return GPIO_NUM_NC;
    }
}

int board_detect_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOARD_SLOT_1_ID_GPIO) |
                        (1ULL << BOARD_SLOT_2_ID_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&cfg);
}

board_slot_info_t board_detect_read_slot(board_slot_t slot)
{
    board_slot_info_t info = {
        .slot = slot,
        .card = BOARD_CARD_UNKNOWN,
        .present = false,
        .id_level = -1,
    };

    gpio_num_t gpio = slot_gpio(slot);
    if (gpio == GPIO_NUM_NC) {
        return info;
    }

    info.id_level = gpio_get_level(gpio);

    /*
     * GPIO39/GPIO42 are digital-only pins on ESP32-S3. With the current
     * hardware we can detect that a card pulls the ID pin low, but cannot
     * measure the resistor value. The magnetic board's 20 kOhm ID resistor
     * is therefore treated as the known low-level card signature.
     */
    if (info.id_level == 0) {
        info.present = true;
        info.card = BOARD_CARD_MAGNETIC;
    } else {
        info.card = BOARD_CARD_NONE;
    }

    return info;
}

uint32_t board_detect_magnetic_slot_mask(void)
{
    uint32_t mask = 0U;

    for (board_slot_t slot = BOARD_SLOT_1; slot < BOARD_SLOT_COUNT; ++slot) {
        board_slot_info_t info = board_detect_read_slot(slot);
        if (info.card == BOARD_CARD_MAGNETIC) {
            mask |= (1UL << slot);
        }
    }

    return mask;
}

const char *board_card_type_name(board_card_type_t card)
{
    switch (card) {
        case BOARD_CARD_NONE:     return "none";
        case BOARD_CARD_MAGNETIC: return "magnetic";
        case BOARD_CARD_UNKNOWN:  return "unknown";
        default:                  return "invalid";
    }
}
