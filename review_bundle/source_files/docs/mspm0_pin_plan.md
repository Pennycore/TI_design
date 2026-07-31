# LP-MSPM0G3507 当前引脚分配

当前以 `car_control/` 为主工程。这个工程已经包含 `car_control.syscfg`，实际引脚以该文件为准。

## 已在 car_control 中配置的引脚

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 | 方向 |
|---|---|---|---|
| 左电机 AIN1 | PB4 | GPIO_MOTOR_DIR_AIN1 | 输出 |
| 左电机 AIN2 | PB0 | GPIO_MOTOR_DIR_AIN2 | 输出 |
| 右电机 BIN1 | PB12 | GPIO_MOTOR_DIR_BIN1 | 输出 |
| 右电机 BIN2 | PB13 | GPIO_MOTOR_DIR_BIN2 | 输出 |
| 电机 STBY | PB3 | GPIO_MOTOR_STBY_STBY | 输出 |
| 左电机 PWM | PA8 | PWM_MOTOR C0 | 输出 |
| 右电机 PWM | PA9 | PWM_MOTOR C1 | 输出 |
| 左编码器 A | PB7 | GPIO_ENCODER_LEFT_A | 输入，上拉，双边沿中断 |
| 左编码器 B | PB6 | GPIO_ENCODER_LEFT_B | 输入，上拉，双边沿中断 |
| 右编码器 A | PB9 | GPIO_ENCODER_RIGHT_A | 输入，上拉，双边沿中断 |
| 右编码器 B | PB8 | GPIO_ENCODER_RIGHT_B | 输入，上拉，双边沿中断 |
| 灰度 CLK | PB15 | GPIO_GRAY_CLK | 推挽输出 |
| 灰度 DAT | PB14 | GPIO_GRAY_DAT | 输入，上拉 |

## 灰度传感器方案

`car_control` 当前使用感为手册第 6 章的 CLK/DAT 串行通讯，不是 UART，也不是 I2C。

| 灰度传感器 | MSPM0G3507 |
|---|---|
| CLK | PB15 |
| DAT | PB14 |
| +5V | 稳定 5V |
| GND | GND |

注意：MSPM0 是 3.3V IO。灰度传感器接线前要插上 `PULL` 开漏模式跳线帽，然后重新给传感器上电；PB14/DAT 靠 MSPM0 内部上拉到 3.3V，不要把 5V DAT 直接输入 MSPM0。

AIN2 原来用 PB5，远端最新工程已改为 PB0，原因是 PB0 可以直接从 BoosterPack 9 号位引出，接线更方便。

## K230 还未合入 car_control

K230 视觉脚本和串口协议已经在 `k230/`、`docs/serial_protocol.md` 里，但 `car_control.syscfg` 目前还没有配置 K230 UART。

由于 PB15/PB14 已经给灰度传感器使用，K230 不能再用我之前临时建议的 PB15/PB16。下一步建议给 K230 单独加一组空闲 UART，例如：

```text
UART2_TX -> PB17 -> 接 K230 RX
UART2_RX -> PB18 -> 接 K230 TX
GND      -> GND
baud     -> 115200 8N1
```

PB17/PB18 在芯片数据手册中支持 UART2_TX/UART2_RX；实际从 LaunchPad 哪个扩展排针引出，需要按板上 J23-J28 或 TI 用户指南再核对。

## 旧方案说明

之前生成的 `firmware/mspm0/` 是一套 HAL 骨架，灰度使用 I2C 模拟量方案。现在有了更完整的 `car_control/`，应优先使用 `car_control/` 上板。`firmware/mspm0/` 可以保留为后续 I2C/视觉融合的参考，不作为当前烧录主工程。
