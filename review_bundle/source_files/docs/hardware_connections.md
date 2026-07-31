# 接线记录

## 总体原则

- 所有模块必须共地：MSPM0、K230、灰度传感器、TB6612FNG、电池负极共地。
- K230 GPIO 和 MSPM0 GPIO 都按 3.3V 逻辑处理，不要直接接 5V 信号。
- 灰度传感器推荐 5V 稳定供电，尽量不要和电机共用未经滤波的 5V。
- TB6612FNG 的 `VM` 用电机电源，`VCC` 用逻辑电源，二者不要接反。

## K230 到 MSPM0 UART

注意：当前 `car_control` 工程还没有合入 K230 UART。PB15/PB14 已经用于灰度传感器 CLK/DAT，不能再分给 K230。

下一步建议在 `car_control.syscfg` 中新增 UART2，并使用 PB17/PB18：

| K230 40Pin | K230 GPIO | 复用功能 | 连接到 MSPM0 |
|---|---:|---|---|
| Pin 11 | GPIO05 | UART2_TXD | MSPM0 PB18 / UART2_RX |
| Pin 13 | GPIO06 | UART2_RXD | MSPM0 PB17 / UART2_TX |
| 任意 GND | GND | GND | GND |

注意：

- 串口交叉连接：K230 TX 接 MSPM0 RX，K230 RX 接 MSPM0 TX。
- 波特率默认 `115200 8N1`。
- K230 建议独立供电或用开发板自己的 USB/5V 输入，MSPM0 不负责给 K230 供电。
- 如果你改用 UART1/UART3/UART4，只需要同步修改 `k230/ball_detect_canmv.py` 和 MSPM0 的 SysConfig。

## 灰度传感器到 MSPM0

| 灰度传感器 | 连接到 MSPM0 | 备注 |
|---|---|---|
| +5V | 稳定 5V | 官方推荐 5V |
| GND | GND | 必须共地 |
| CLK | PB15 | 推挽输出 |
| DAT | PB14 | 输入，上拉到 3.3V |
| KEY | 可不接或外接按键到 GND | 用于传感器校准 |
| ERR | 可选 GPIO 输入 | 调试错误状态 |

当前 `car_control` 使用手册第 6 章的 CLK/DAT 串行通讯。传感器接线前要插上 `PULL` 开漏模式跳线帽，然后重新给传感器上电；DAT 由 MSPM0 内部上拉到 3.3V。

## TB6612FNG 到 MSPM0

| TB6612FNG | 连接到 MSPM0 | 用途 |
|---|---|---|
| PWMA | PA8 | 左电机 PWM |
| AIN1 | PB4 | 左电机方向 |
| AIN2 | PB0 | 左电机方向 |
| PWMB | PA9 | 右电机 PWM |
| BIN1 | PB12 | 右电机方向 |
| BIN2 | PB13 | 右电机方向 |
| STBY | PB3 | 高电平使能 |
| VCC | 3.3V | 逻辑电源 |
| VM | 电机电源 | 按电机额定电压 |
| GND | GND | 必须共地 |

MSPM0 详细引脚计划见 `docs/mspm0_pin_plan.md`。当前上板主工程是 `car_control/`。

## 灰度传感器现场校准

每次改变安装高度后重新校准。手册建议探头距地约 20 mm 起步，黑白场无先后顺序。输出逻辑为：

- 接近白场：数字量为 1。
- 接近黑场：数字量为 0。
- 归一化模拟量：白场约 255，黑场约 0。
