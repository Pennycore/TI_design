# LP-MSPM0G3507 LaunchPad 小车控制工程

本目录是从原 MSPM0G3507 小车工程迁移得到的完整 TI CCS/SysConfig 工程，
目标硬件为 TI `LP-MSPM0G3507` LaunchPad。由于主控芯片和 LQFP64 封装没有变化，
电机、编码器、灰度传感器和循迹算法代码均可直接复用。

迁移时只调整了电机 `STBY`：原工程使用 `PA10`，LaunchPad 默认可通过 J21
将 PA10 连接到板载 XDS110 串口，因此改用空闲的 `PB3`，避免与调试通道互相影响。

本工程目前包含：

- TB6612 双路电机驱动
- 左右轮 AB 相编码器四倍频计数
- 双轮 10 ms 速度闭环
- 感为八路灰度传感器 CLK/DAT 串行读取
- 八路数字量加权位置计算和循迹 PID

## 引脚分配

| 功能 | MSPM0G3507 引脚 | 方向 |
| --- | --- | --- |
| 左电机 AIN1 | PB4 | 输出 |
| 左电机 AIN2 | PB0（BoosterPack 9 号位） | 输出 |
| 右电机 BIN1 | PB12 | 输出 |
| 右电机 BIN2 | PB13 | 输出 |
| 电机 STBY | PB3 | 输出 |
| 左电机 PWM | PA8 | 输出 |
| 右电机 PWM | PA9 | 输出 |
| 左编码器 A 相 | PB7 | 输入，上拉，双边沿中断 |
| 左编码器 B 相 | PB6 | 输入，上拉，双边沿中断 |
| 右编码器 A 相 | PB9 | 输入，上拉，双边沿中断 |
| 右编码器 B 相 | PB8 | 输入，上拉，双边沿中断 |
| 灰度传感器 AD0 | PB15 | 推挽输出 |
| 灰度传感器 AD1 | PB2（BoosterPack 常用排针） | 推挽输出 |
| 灰度传感器 AD2 | PB16 | 推挽输出 |
| 灰度传感器 OUT | PA27 / ADC0.0 | 模拟输入 |

以上为 MCU 引脚名，不是 LaunchPad 排针编号。所有信号都可以从板上的
BoosterPack 排针或 J23～J28 MCU 引脚扩展排针引出。

原工程的 AIN2 使用 PB5，但 PB5 只在底部 MCU 扩展排针上，不方便接线。
TI LaunchPad 版本改用 40Pin BoosterPack 排针上的 PB0（9 号位）。

右电机 PWM 使用 `PA9`。若从 BoosterPack 排针引出 PA9，需要将 J14
从默认的 PB23 位置切换到 PA9；若直接使用 MCU 引脚扩展排针，则无需修改 J14。

## 灰度传感器接线

灰度传感器为无 MCU 模拟量版本。AD0～AD2 选择 74HC4051 的一路探头，
OUT 由 MSPM0G3507 的 ADC 读取。

| 灰度传感器 | MSPM0G3507 |
| --- | --- |
| AD0 | PB15 |
| AD1 | PB2 |
| AD2 | PB16 |
| OUT | PA27 / ADC0.0 |
| EN | 悬空（模块内部 10 kΩ 下拉，低电平使能） |
| ERR | 悬空，暂不使用 |
| GND | GND（必须共地） |
| +5V | 稳定 5V 电源 |

模块工作电压为 5～12 V，典型电流约 120 mA。建议使用独立、稳定的 5 V
稳压电源，不与电机直接共用未经滤波的电源。LaunchPad、灰度传感器、
TB6612 和电机电源负极必须共地。

PA27 的 ADC 绝对不能输入超过 3.3 V。首次连接前先用万用表确认 OUT
最高电压；不确定时使用 10 kΩ 上臂、20 kΩ 下臂的分压器。

## 灰度数据

`GraySensor_Read()` 依次选择 CH1～CH8、ADC 采样并进行滞回比较，最后返回
与原循迹层兼容的八路数字量：

- bit0～bit7 对应第 1～8 路探头
- 接近白场输出 1
- 接近黑场输出 0

`g_graySensorRaw[8]` 可在 CCS 中观察每路 12 位 ADC 原始值。
`g_graySensorWhite[8]` 和 `g_graySensorBlack[8]` 必须使用最终安装高度下的
实际白场、黑场数据替换。传感器改变安装高度后需要重新校准。

## 循迹控制

`LineControl_Update()` 每次读取八路数据，计算黑线相对车身中心的位置，
再通过循迹 PID 修正左右轮目标速度：

```text
左轮目标 = 基础速度 + 转向修正
右轮目标 = 基础速度 - 转向修正
```

默认配置位于 `line_control.h`：

- `LINE_CONTROL_DEFAULT_BASE_SPEED`：直线基础速度
- `LINE_CONTROL_KP/KI/KD`：循迹 PID
- `LINE_CONTROL_BIT0_IS_LEFT`：探头顺序
- `LINE_CONTROL_TRACK_BLACK_LINE`：循黑线或循白线

首次测试时，可调用 `LineControl_GetStatus()` 获取 `LineControl_Status_t`，
也可以在调试器中查看 `line_control.c` 内的 `g_status`：

- `raw_sensor`：传感器原始数据
- `line_bits`：转换后的目标线数据
- `position`：目标线位置，负数在左，正数在右
- `left_target/right_target`：左右轮目标速度
- `lost_count`：未检测到线的累计采样次数

未检测到黑线时程序默认停车。实测确认探头方向、转向方向和赛道形式后，
再增加丢线搜索、十字路口和特殊元素处理。

## 调试注意事项

首次测试电机闭环时请架空车轮。当前 PID 参数和目标速度只是低速起步值，
需要根据电机、减速比、编码器分辨率和车体负载继续整定。

建议按以下顺序上电检查：

1. 不连接电机 VM，只通过 USB 给 LaunchPad 供电，确认可以下载和调试。
2. 检查灰度传感器 DAT 在 3.3V 逻辑范围内，并确认八路数据变化正常。
3. 架空车轮后连接电机电源，确认左右电机和编码器方向。
4. 将小车放到赛道上，以默认低速开始调整循迹 PID。

`应用手册.pdf` 是软件定义玻璃 LCD 应用笔记，与本小车工程无关。
芯片引脚和电气限制应以 MSPM0G3507 数据手册及 LP-MSPM0G3507 用户指南为准。
