#include "gw_gray.h"

#include <stddef.h>
#include <string.h>

#include "hal.h"

#define GW_CMD_ANALOG_CONTINUOUS 0xB0U
#define GW_CMD_DIGITAL 0xDDU
#define GW_CMD_CHANNEL_ENABLE 0xCEU
#define GW_CMD_NORMALIZATION 0xCFU
#define GW_CMD_PING 0xAAU
#define GW_CMD_ERROR 0xDEU
#define GW_CMD_VERSION 0xC1U
#define GW_PING_REPLY 0x66U

static bool version_at_least(uint8_t version, uint8_t major, uint8_t minor)
{
    uint8_t v_major = (uint8_t)(version >> 4);
    uint8_t v_minor = (uint8_t)(version & 0x0F);

    if (v_major != major) {
        return v_major > major;
    }
    return v_minor >= minor;
}

static bool write_command(gw_gray_t *sensor, uint8_t cmd)
{
    return hal_i2c_write(HAL_I2C_GRAY, sensor->addr7, &cmd, 1U, true);
}

bool gw_gray_ping(gw_gray_t *sensor)
{
    uint8_t cmd = GW_CMD_PING;
    uint8_t reply = 0U;

    if (!hal_i2c_write_read(HAL_I2C_GRAY, sensor->addr7, &cmd, 1U, &reply, 1U)) {
        return false;
    }

    return reply == GW_PING_REPLY;
}

bool gw_gray_read_version(gw_gray_t *sensor, uint8_t *version)
{
    uint8_t cmd = GW_CMD_VERSION;
    uint8_t value = 0U;

    if (!hal_i2c_write_read(HAL_I2C_GRAY, sensor->addr7, &cmd, 1U, &value, 1U)) {
        return false;
    }

    *version = value;
    sensor->version = value;
    return true;
}

bool gw_gray_read_digital(gw_gray_t *sensor, uint8_t *digital_bits)
{
    uint8_t cmd = GW_CMD_DIGITAL;
    return hal_i2c_write_read(HAL_I2C_GRAY, sensor->addr7, &cmd, 1U, digital_bits, 1U);
}

bool gw_gray_read_analog8(gw_gray_t *sensor, uint8_t analog[ROBOT_GRAY_SENSOR_COUNT])
{
    uint8_t cmd = GW_CMD_ANALOG_CONTINUOUS;
    return hal_i2c_write_read(HAL_I2C_GRAY,
                              sensor->addr7,
                              &cmd,
                              1U,
                              analog,
                              ROBOT_GRAY_SENSOR_COUNT);
}

bool gw_gray_read_error(gw_gray_t *sensor, uint8_t *error_flags)
{
    uint8_t cmd = GW_CMD_ERROR;
    return hal_i2c_write_read(HAL_I2C_GRAY, sensor->addr7, &cmd, 1U, error_flags, 1U);
}

bool gw_gray_set_channel_enable(gw_gray_t *sensor, uint8_t mask)
{
    uint8_t packet[2] = {GW_CMD_CHANNEL_ENABLE, mask};
    return hal_i2c_write(HAL_I2C_GRAY, sensor->addr7, packet, sizeof(packet), true);
}

bool gw_gray_set_normalization(gw_gray_t *sensor, uint8_t mask)
{
    uint8_t packet[2] = {GW_CMD_NORMALIZATION, mask};
    bool ok = hal_i2c_write(HAL_I2C_GRAY, sensor->addr7, packet, sizeof(packet), true);
    sensor->normalization_enabled = ok && (mask != 0U);
    return ok;
}

bool gw_gray_init(gw_gray_t *sensor)
{
    static const uint8_t candidate_addrs[] = {
        ROBOT_GRAY_I2C_ADDR_PREFERRED,
        ROBOT_GRAY_I2C_ADDR_MIN,
        (uint8_t)(ROBOT_GRAY_I2C_ADDR_MIN + 1U),
        (uint8_t)(ROBOT_GRAY_I2C_ADDR_MIN + 2U),
        ROBOT_GRAY_I2C_ADDR_MAX,
    };

    memset(sensor, 0, sizeof(*sensor));

    for (size_t i = 0U; i < (sizeof(candidate_addrs) / sizeof(candidate_addrs[0])); ++i) {
        uint8_t addr = candidate_addrs[i];
        bool already_tried = false;

        for (size_t j = 0U; j < i; ++j) {
            if (candidate_addrs[j] == addr) {
                already_tried = true;
                break;
            }
        }
        if (already_tried) {
            continue;
        }

        sensor->addr7 = addr;
        for (uint8_t retry = 0U; retry < 10U; ++retry) {
            if (gw_gray_ping(sensor)) {
                (void)gw_gray_read_version(sensor, &sensor->version);
                (void)gw_gray_set_channel_enable(sensor, 0xFFU);
                if (version_at_least(sensor->version, 3U, 6U)) {
                    (void)gw_gray_set_normalization(sensor, 0xFFU);
                }
                (void)write_command(sensor, GW_CMD_ANALOG_CONTINUOUS);
                return true;
            }
            hal_delay_ms(20U);
        }
    }

    sensor->addr7 = 0U;
    return false;
}

bool gw_gray_read_sample(gw_gray_t *sensor, gw_gray_sample_t *sample)
{
    memset(sample, 0, sizeof(*sample));

    sample->analog_valid = gw_gray_read_analog8(sensor, sample->analog);
    sample->digital_valid = gw_gray_read_digital(sensor, &sample->digital_bits);
    sample->error_valid = gw_gray_read_error(sensor, &sample->error_flags);

    return sample->analog_valid || sample->digital_valid;
}
