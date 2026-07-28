# car_control 下一步操作

现在主工程是 `car_control/`。先不要管 `firmware/mspm0/`，那是早期 HAL 骨架。

## 你现在要做的事

1. 用 CCS / Theia 打开或导入 `car_control/`。
2. 打开 `car_control/car_control.syscfg`，确认没有红色冲突。
3. Build 工程，确认 SysConfig 能生成 `ti_msp_dl_config.h/.c`。
4. 先只用 USB 给 LaunchPad 供电，不接电机 VM，下载程序。
5. 接灰度传感器，确认 PB14/DAT 是 3.3V 逻辑。
6. 架空车轮，再接 TB6612 和电机 VM。
7. 低速调试编码器方向、电机方向、循迹方向。

## 第一阶段只跑这些

- TB6612 双电机控制
- 编码器测速
- 10 ms 速度闭环
- 灰度 CLK/DAT 读取
- 黑线循迹 PID

## K230 下一阶段再接

K230 视觉识别目前还没有并入 `car_control.syscfg`。建议下一步新增：

```text
UART2_TX -> PB17 -> K230 RX
UART2_RX -> PB18 -> K230 TX
baud     -> 115200 8N1
```

然后把 `firmware/mspm0/src/serial_protocol.c/.h` 的协议代码迁移到 `car_control/`，在主循环中解析 K230 的 `VISION_BALL` 帧。
