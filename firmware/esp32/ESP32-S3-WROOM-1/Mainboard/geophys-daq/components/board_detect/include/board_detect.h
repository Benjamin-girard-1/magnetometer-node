/**
 * @file board_detect.h
 * @brief Expansion-card presence and ID detection.
 */

#ifndef BOARD_DETECT_H_
#define BOARD_DETECT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_SLOT_1 = 0,
    BOARD_SLOT_2 = 1,
    BOARD_SLOT_COUNT,
} board_slot_t;

typedef enum {
    BOARD_CARD_NONE = 0,
    BOARD_CARD_MAGNETIC,
    BOARD_CARD_UNKNOWN,
} board_card_type_t;

typedef struct {
    board_slot_t slot;
    board_card_type_t card;
    bool present;
    int id_level;
} board_slot_info_t;

int board_detect_init(void);
board_slot_info_t board_detect_read_slot(board_slot_t slot);
uint32_t board_detect_magnetic_slot_mask(void);
const char *board_card_type_name(board_card_type_t card);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_DETECT_H_ */
