# 从原控制板迁移到 TI LaunchPad

## 迁移结论

原工程和 TI LaunchPad 都使用 MSPM0G3507 LQFP64，因此算法层和 DriverLib
调用不需要重写。迁移后的工程保留：

- TB6612 双电机控制
- 左右编码器 AB 相四倍频计数
- 10 ms 双轮速度闭环
- 感为八路灰度 CLK/DAT 读取
- 灰度加权位置计算和循迹 PID

## 唯一的软件引脚修改

| 信号 | 原引脚 | TI LaunchPad 工程 |
| --- | --- | --- |
| TB6612 STBY | PA10 | PB3 |

PA10 在 LaunchPad 上默认可通过 J21 接到 XDS110 的 UART 通道。改到 PB3 后，
无需拔掉 J21，也不会让 STBY 输出干扰板载调试串口。

## 不需要修改的代码

以下文件与开发板载板无关，已经直接复用：

- `encoder.c/.h`
- `gray_sensor.c/.h`
- `line_control.c/.h`
- `motor.c/.h`
- `pid.c/.h`
- `speed_control.c/.h`

所有实际引脚均由 `car_control.syscfg` 生成宏，业务代码不硬编码端口号。

## 上板前必须确认

- 感为灰度传感器安装 PULL 开漏跳线帽，并在安装后重新上电。
- PB14/DAT 使用 3.3V 上拉，禁止直接输入 5V。
- PA9 从 BoosterPack 排针使用时，将 J14 选择到 PA9。
- 电机 VM 使用独立电源，不从 LaunchPad 5V 排针供电。
- LaunchPad、灰度传感器、TB6612 和电机电源必须共地。
