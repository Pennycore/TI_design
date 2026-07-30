# 球杆硬件接口（MSPM0G3507，LQFP-64）

本表依据 MS42CG、D36A 和 MSPM0G3507 手册确定，并避开现有小车的
TIMA0、TIMA1、UART2、车轮编码器和灰度传感器引脚。

| 模块 | 商家端子 | MSPM0G3507 | 外设功能 |
|---|---|---|---|
| D36A 通道 1 | STEP1 | PA23 | TIMG7_CCP0，步进脉冲 |
| D36A 通道 1 | DIR1 | PA13 | GPIO 输出 |
| D36A 通道 1 | EN1 | PA12 | GPIO 输出，高电平使能 |
| MS42CG 编码器 | A | PA1 | TIMG8_CCP0，QEI A |
| MS42CG 编码器 | B | PA0 | TIMG8_CCP1，QEI B |
| MS42CG 编码器 | PWM | PB20 | TIMG12_CCP0，绝对角 PWM 捕获 |
| MS42CG 编码器 | Z | PA25 | GPIO 输入，单圈零位脉冲 |

## 接线和供电注意事项

- MS42CG 六芯编码器插座按手册视图依次为 `GND、Z、PWM、B、A、VCC`。
  编码器 VCC 接 3.3 V，使 A/B/PWM/Z 输出电平天然兼容 MSPM0G3507。
- D36A 的 VIN 接电机电源（手册示例为 12 V），D36A GND 与单片机 GND
  必须共地。D36A 板载 `5V` 是输出端，不要接到 MSPM0 的 3.3 V 电源。
- STEP1、DIR1、EN1 只接 D36A 通道 1；未使用的通道 2 可悬空。
- 16 细分时，D36A 的拨码 1/2/3 均为 OFF。初次上电建议先用最低电流档
  （拨码 4/5/6 均为 ON，手册标称约 0.55 A），确认温升和力矩后再调整。
- PA23 同时具备 VREF+ 复用功能，本工程将其用作 TIMG7 步进脉冲；
  不要再外接 VREF+。

## 软件资源

- `stepper_motor.c/.h`：D36A 的脉冲、方向、使能，以及指定步数自动停止。
- `stepper_encoder.c/.h`：MS42CG 的 4000 计数/圈 AB 闭环、PWM 绝对角和
  Z 脉冲计数。

全局 `SYSCFG_DL_init()` 之后依次调用：

```c
StepperMotor_Init();
StepperEncoder_Init();
```

闭环控制周期内调用 `StepperEncoder_Update()`，再读取编码器计数或角度。
