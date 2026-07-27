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
| 灰度传感器 CLK | PB15 | 推挽输出 |
| 灰度传感器 DAT | PB14 | 上拉输入 |

以上为 MCU 引脚名，不是 LaunchPad 排针编号。所有信号都可以从板上的
BoosterPack 排针或 J23～J28 MCU 引脚扩展排针引出。

原工程的 AIN2 使用 PB5，但 PB5 只在底部 MCU 扩展排针上，不方便接线。
TI LaunchPad 版本改用 40Pin BoosterPack 排针上的 PB0（9 号位）。

右电机 PWM 使用 `PA9`。若从 BoosterPack 排针引出 PA9，需要将 J14
从默认的 PB23 位置切换到 PA9；若直接使用 MCU 引脚扩展排针，则无需修改 J14。

## 灰度传感器接线

灰度传感器采用手册中的 CLK/DAT 串行输出方式，不是 UART。

| 灰度传感器 | MSPM0G3507 |
| --- | --- |
| CLK | PB15 |
| DAT | PB14 |
| GND | GND（必须共地） |
| +5V | 稳定 5V 电源 |

MSPM0G3507 使用 3.3V IO，PB14 是普通 IO，不能直接承受 5V DAT。
传感器连接前应安装 `PULL` 开漏模式跳线帽，
然后重新给传感器上电；DAT 由主控内部上拉至 3.3V。CLK 保持推挽输出。

灰度传感器可以使用 LaunchPad J10 的 5V 供电，但电机 VM 必须使用独立的
电机电源。LaunchPad、灰度传感器、TB6612 和电机电源负极必须共地。

## 灰度数据

`GraySensor_Read()` 返回八路数字量：

- bit0～bit7 对应第 1～8 路探头
- 接近白场输出 1
- 接近黑场输出 0

函数每次读取前会等待 1 ms 进行帧同步，CLK 高电平保持至少 5 us。
传感器改变安装高度后需要重新校准。

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
