#include "hal.h"

#include <string.h>

static uint32_t g_now_ms;

void hal_init(void)
{
    g_now_ms = 0U;
}

uint32_t hal_millis(void)
{
    return g_now_ms;
}

void hal_delay_ms(uint32_t ms)
{
    g_now_ms += ms;
}

bool hal_i2c_write(hal_i2c_t bus, uint8_t addr7, const uint8_t *data, size_t len, bool stop)
{
    (void)bus;
    (void)addr7;
    (void)data;
    (void)len;
    (void)stop;
    return true;
}

bool hal_i2c_read(hal_i2c_t bus, uint8_t addr7, uint8_t *data, size_t len)
{
    (void)bus;
    (void)addr7;
    memset(data, 0xFF, len);
    return true;
}

bool hal_i2c_write_read(hal_i2c_t bus,
                        uint8_t addr7,
                        const uint8_t *tx,
                        size_t tx_len,
                        uint8_t *rx,
                        size_t rx_len)
{
    (void)bus;
    (void)addr7;
    if ((tx_len == 1U) && (tx[0] == 0xAAU) && (rx_len == 1U)) {
        rx[0] = 0x66U;
        return true;
    }
    memset(rx, 0xFF, rx_len);
    return true;
}

size_t hal_uart_read(hal_uart_t uart, uint8_t *data, size_t max_len)
{
    (void)uart;
    (void)data;
    (void)max_len;
    return 0U;
}

size_t hal_uart_write(hal_uart_t uart, const uint8_t *data, size_t len)
{
    (void)uart;
    (void)data;
    return len;
}

void hal_gpio_write(hal_gpio_t gpio, bool high)
{
    (void)gpio;
    (void)high;
}

void hal_pwm_write(hal_pwm_t pwm, int16_t duty_per_mille)
{
    (void)pwm;
    (void)duty_per_mille;
}

void hal_log(const char *text)
{
    (void)text;
}
