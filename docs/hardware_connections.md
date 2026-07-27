# 接线记录

## 总体原则

- 所有模块必须共地：MSPM0、K230、灰度传感器、TB6612FNG、电池负极共地。
- K230 GPIO 和 MSPM0 GPIO 都按 3.3V 逻辑处理，不要直接接 5V 信号。
- 灰度传感器推荐 5V 稳定供电，尽量不要和电机共用未经滤波的 5V。
- TB6612FNG 的 `VM` 用电机电源，`VCC` 用逻辑电源，二者不要接反。

## K230 到 MSPM0 UART

根据你给的庐山派 Lite K230D 40Pin 图，推荐用 UART2：

| K230 40Pin | K230 GPIO | 复用功能 | 连接到 MSPM0 |
|---|---:|---|---|
| Pin 11 | GPIO05 | UART2_TXD | K230_UART_RX |
| Pin 13 | GPIO06 | UART2_RXD | K230_UART_TX |
| 任意 GND | GND | GND | GND |

注意：

- 串口交叉连接：K230 TX 接 MSPM0 RX，K230 RX 接 MSPM0 TX。
- 波特率默认 `115200 8N1`。
- K230 建议独立供电或用开发板自己的 USB/5V 输入，MSPM0 不负责给 K230 供电。
- 如果你改用 UART1/UART3/UART4，只需要同步修改 `k230/ball_detect_canmv.py` 和 MSPM0 的 SysConfig。

## 灰度传感器到 MSPM0 I2C

| 灰度传感器 | 连接到 MSPM0 | 备注 |
|---|---|---|
| +5V | 稳定 5V | 官方推荐 5V |
| GND | GND | 必须共地 |
| SCL | GRAY_I2C_SCL | 建议上拉到 3.3V |
| SDA | GRAY_I2C_SDA | 建议上拉到 3.3V |
| KEY | 可不接或外接按键到 GND | 用于传感器校准 |
| ERR | 可选 GPIO 输入 | 调试错误状态 |

地址说明：

- 传感器软件地址高 5 位默认 `10011`。
- AD1/AD0 未插跳线帽时，7 位地址通常是 `0x4C`。
- AD1/AD0 都插上时，7 位地址是 `0x4F`，手册示例也常用 `0x4F`。
- MSPM0 代码会优先试配置值，再扫描 `0x4C` 到 `0x4F`。

I2C 上拉注意：

- 灰度传感器 I2C 的板载上拉跳线是 5V/10k，应先确认 MSPM0 引脚是否 5V 容忍。
- 更稳妥做法：不插 5V 上拉跳线，外接 2.2k 到 4.7k 上拉到 3.3V。
- 手册写明 I2C 时无需进入开漏模式；开漏跳线主要用于并口或传感器自定义串行 DAT。

## TB6612FNG 到 MSPM0

| TB6612FNG | 连接到 MSPM0 | 用途 |
|---|---|---|
| PWMA | MOTOR_LEFT_PWM | 左电机 PWM |
| AIN1 | MOTOR_LEFT_IN1 | 左电机方向 |
| AIN2 | MOTOR_LEFT_IN2 | 左电机方向 |
| PWMB | MOTOR_RIGHT_PWM | 右电机 PWM |
| BIN1 | MOTOR_RIGHT_IN1 | 右电机方向 |
| BIN2 | MOTOR_RIGHT_IN2 | 右电机方向 |
| STBY | MOTOR_STBY | 高电平使能 |
| VCC | 3.3V | 逻辑电源 |
| VM | 电机电源 | 按电机额定电压 |
| GND | GND | 必须共地 |

MSPM0 具体引脚请在 TI SysConfig 中绑定到上面的逻辑名。`firmware/mspm0/include/hal.h` 已经把这些逻辑名抽象出来。

## 灰度传感器现场校准

每次改变安装高度后重新校准。手册建议探头距地约 20 mm 起步，黑白场无先后顺序。输出逻辑为：

- 接近白场：数字量为 1。
- 接近黑场：数字量为 0。
- 归一化模拟量：白场约 255，黑场约 0。
