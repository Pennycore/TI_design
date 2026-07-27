#include "hal.h"

/*
 * This is the hardware binding template for LP-MSPM0G3507.
 *
 * Replace each function with your TI DriverLib/SysConfig calls:
 * - I2C master: GW gray sensor, 100 kHz or 400 kHz.
 * - UART: K230, 115200 8N1.
 * - PWM: TB6612FNG PWMA/PWMB.
 * - GPIO: AIN1/AIN2/BIN1/BIN2/STBY.
 */

void hal_init(void)
{
    /* Call SYSCFG_DL_init(); start timers/PWM; enable UART RX interrupt if used. */
}

uint32_t hal_millis(void)
{
    /* Return a 1 ms system tick. */
    return 0U;
}

void hal_delay_ms(uint32_t ms)
{
    (void)ms;
}

bool hal_i2c_write(hal_i2c_t bus, uint8_t addr7, const uint8_t *data, size_t len, bool stop)
{
    (void)bus;
    (void)addr7;
    (void)data;
    (void)len;
    (void)stop;
    return false;
}

bool hal_i2c_read(hal_i2c_t bus, uint8_t addr7, uint8_t *data, size_t len)
{
    (void)bus;
    (void)addr7;
    (void)data;
    (void)len;
    return false;
}

bool hal_i2c_write_read(hal_i2c_t bus,
                        uint8_t addr7,
                        const uint8_t *tx,
                        size_t tx_len,
                        uint8_t *rx,
                        size_t rx_len)
{
    if (!hal_i2c_write(bus, addr7, tx, tx_len, false)) {
        return false;
    }
    return hal_i2c_read(bus, addr7, rx, rx_len);
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
