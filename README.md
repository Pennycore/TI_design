# TI Design Robot Baseline

这是面向电赛小车的一版起步工程，硬件分工如下：

- LP-MSPM0G3507：主控，负责灰度传感器读取、循迹 PID、TB6612FNG 电机驱动、状态机和串口协议。
- 庐山派 Lite K230D / CanMV：视觉协处理器，负责小钢球识别，把目标位置通过 UART 发给 MSPM0。
- 感为 8 路灰度传感器：建议通过 I2C 读取 8 路归一化模拟量，保留数字量读取做调试。
- TB6612FNG：双路直流电机驱动，PWM + 方向控制。

## 目录

- `car_control/`：当前主用的 LP-MSPM0G3507 CCS/SysConfig 小车控制工程。
- `firmware/mspm0/`：早期生成的 MSPM0 HAL 骨架，保留作后续 I2C/视觉融合参考。
- `k230/`：CanMV/K230 小钢球识别脚本。
- `vision/`：YOLO11n 小钢球检测训练、数据集和导出工程。
- `docs/`：算法选择、串口协议、接线记录。
- `tools/`：电脑端串口调试工具。

## 当前算法选择

当前上板先使用 `car_control/` 的“灰度数字量循迹 + 编码器速度闭环”。K230 小钢球识别脚本已经在 `k230/`，但还未合入 `car_control` 的 UART 外设配置。

## 代码落地方式

优先把 `car_control/car_control.syscfg` 作为 CCS/SysConfig 工程打开并编译。后续再把 K230 UART 和 `docs/serial_protocol.md` 中的视觉协议接入 `car_control`。

推荐先看：

1. `docs/hardware_connections.md`
2. `docs/serial_protocol.md`
3. `firmware/mspm0/README.md`
4. `k230/README.md`
