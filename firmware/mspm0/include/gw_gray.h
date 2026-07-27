#ifndef GW_GRAY_H
#define GW_GRAY_H

#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"

typedef struct {
    uint8_t addr7;
    uint8_t version;
    bool normalization_enabled;
} gw_gray_t;

typedef struct {
    uint8_t digital_bits;
    uint8_t analog[ROBOT_GRAY_SENSOR_COUNT];
    uint8_t error_flags;
    bool digital_valid;
    bool analog_valid;
    bool error_valid;
} gw_gray_sample_t;

bool gw_gray_init(gw_gray_t *sensor);
bool gw_gray_ping(gw_gray_t *sensor);
bool gw_gray_read_version(gw_gray_t *sensor, uint8_t *version);
bool gw_gray_read_digital(gw_gray_t *sensor, uint8_t *digital_bits);
bool gw_gray_read_analog8(gw_gray_t *sensor, uint8_t analog[ROBOT_GRAY_SENSOR_COUNT]);
bool gw_gray_read_error(gw_gray_t *sensor, uint8_t *error_flags);
bool gw_gray_set_channel_enable(gw_gray_t *sensor, uint8_t mask);
bool gw_gray_set_normalization(gw_gray_t *sensor, uint8_t mask);
bool gw_gray_read_sample(gw_gray_t *sensor, gw_gray_sample_t *sample);

#endif
