# MSPM0 Firmware

这一层代码按“应用逻辑 + HAL 适配”组织：

- `include/hal.h`：需要你在 TI 工程中实现的硬件接口。
- `src/gw_gray.c`：感为 8 路灰度传感器 I2C 驱动。
- `src/line_control.c`：8 路灰度质心估计和 PID。
- `src/motor_control.c`：TB6612FNG 电机输出。
- `src/serial_protocol.c`：K230/MSPM0 二进制串口协议。
- `src/robot_app.c`：整车状态机。
- `port/ti_mspm0_port.c`：TI DriverLib/SysConfig 绑定模板。

## 移植步骤

1. 在 CCS 或 Keil 中创建 LP-MSPM0G3507 工程。
2. 用 SysConfig 配置：
   - 1 路 I2C master，用于灰度传感器。
   - 1 路 UART，用于 K230，115200 8N1。
   - 2 路 PWM，用于 TB6612FNG 的 PWMA/PWMB。
   - 5 个 GPIO 输出，用于 AIN1/AIN2/BIN1/BIN2/STBY。
3. 把 `src/*.c` 和 `include/*.h` 加入工程。
4. 参考 `port/ti_mspm0_port.c` 实现 `hal.h` 中的函数。
5. 把 `src/main.c` 作为应用入口，或将 `robot_app_init/tick/poll_uart` 接入你已有主循环。

## 现场调参优先级

1. `ROBOT_BASE_PWM`：先低速，确认方向后再加速。
2. `ROBOT_LINE_KP/KD`：先只调 P，再加 D 抑制摆动。
3. `ROBOT_BALL_STOP_RADIUS`：钢球靠近到多大时停车。
4. K230 脚本里的 `CIRCLE_THRESHOLD`、`MIN_RADIUS`、`MAX_RADIUS`。
