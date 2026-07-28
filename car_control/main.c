#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "k230_uart.h"
#include "line_control.h"
#include "motor.h"
#include "speed_control.h"

/*
 * GraySensor_Read() 内部等待约 1 ms 完成帧同步，
 * 主循环再等待约 9 ms，使循迹更新周期接近 10 ms。
 */
#define LINE_CONTROL_MAIN_DELAY_CYCLES \
    ((CPUCLK_FREQ / 1000U) * 9U)

#define LINE_CONTROL_STARTUP_DELAY_CYCLES \
    (CPUCLK_FREQ * 3U)

int main(void)
{
    SYSCFG_DL_init();

    /*
     * 速度控制定时器启动前，必须先完成电机和编码器初始化。
     */
    Motor_Init();
    Encoder_Init();
    GraySensor_Init();
    K230Uart_Init();
    SpeedControl_Init();
    LineControl_Init();

    /*
     * Wait three seconds after reset before enabling line tracking so the
     * operator can put the car down and move a hand away safely.
     */
    delay_cycles(LINE_CONTROL_STARTUP_DELAY_CYCLES);

    while (1)
    {
        K230Uart_Poll();
        LineControl_Update();
        K230Uart_Poll();
        delay_cycles(LINE_CONTROL_MAIN_DELAY_CYCLES);
    }
}
