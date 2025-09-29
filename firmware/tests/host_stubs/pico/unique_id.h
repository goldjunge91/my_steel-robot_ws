// Minimal stub for pico/unique_id.h used by host tests.
#pragma once

#include <stdint.h>

#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8

typedef struct pico_unique_board_id_t {
    uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
} pico_unique_board_id_t;

static inline void pico_get_unique_board_id(pico_unique_board_id_t *out) {
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i)
        out->id[i] = (uint8_t)i;
}
