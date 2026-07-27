#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    HAL_GPIO_MOTOR_LEFT_IN1 = 0,
    HAL_GPIO_MOTOR_LEFT_IN2,
    HAL_GPIO_MOTOR_RIGHT_IN1,
    HAL_GPIO_MOTOR_RIGHT_IN2,
    HAL_GPIO_MOTOR_STBY,
} hal_gpio_t;

typedef enum {
    HAL_PWM_MOTOR_LEFT = 0,
    HAL_PWM_MOTOR_RIGHT,
} hal_pwm_t;

typedef enum {
    HAL_I2C_GRAY = 0,
} hal_i2c_t;

typedef enum {
    HAL_UART_K230 = 0,
    HAL_UART_DEBUG,
} hal_uart_t;

void hal_init(void);
uint32_t hal_millis(void);
void hal_delay_ms(uint32_t ms);

bool hal_i2c_write(hal_i2c_t bus, uint8_t addr7, const uint8_t *data, size_t len, bool stop);
bool hal_i2c_read(hal_i2c_t bus, uint8_t addr7, uint8_t *data, size_t len);
bool hal_i2c_write_read(hal_i2c_t bus,
                        uint8_t addr7,
                        const uint8_t *tx,
                        size_t tx_len,
                        uint8_t *rx,
                        size_t rx_len);

size_t hal_uart_read(hal_uart_t uart, uint8_t *data, size_t max_len);
size_t hal_uart_write(hal_uart_t uart, const uint8_t *data, size_t len);

void hal_gpio_write(hal_gpio_t gpio, bool high);
void hal_pwm_write(hal_pwm_t pwm, int16_t duty_per_mille);

void hal_log(const char *text);

#endif
