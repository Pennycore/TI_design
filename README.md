# TI Design Robot Baseline

这是面向电赛小车的一版起步工程，硬件分工如下：

- LP-MSPM0G3507：主控，负责灰度传感器读取、循迹 PID、TB6612FNG 电机驱动、状态机和串口协议。
- 庐山派 Lite K230D / CanMV：视觉协处理器，负责小钢球识别，把目标位置通过 UART 发给 MSPM0。
- 感为 8 路灰度传感器：建议通过 I2C 读取 8 路归一化模拟量，保留数字量读取做调试。
- TB6612FNG：双路直流电机驱动，PWM + 方向控制。

## 目录

- `firmware/mspm0/`：MSPM0 应用代码和可移植 HAL 接口。
- `k230/`：CanMV/K230 小钢球识别脚本。
- `docs/`：算法选择、串口协议、接线记录。
- `tools/`：电脑端串口调试工具。

## 当前算法选择

第一版选择“灰度模拟量质心循迹 + K230 传统视觉圆/亮斑检测”。原因是现场调参快，不需要先采集训练集。若背景复杂、钢球反光严重或目标尺度变化大，再升级为 K230 上的 YOLO/kmodel 检测。

## 代码落地方式

`firmware/mspm0/include/hal.h` 定义了 MSPM0 需要实现的硬件接口。你把工程导入 TI CCS/SysConfig 后，需要在 `firmware/mspm0/port/ti_mspm0_port.c` 里把 I2C、UART、PWM、GPIO 绑定到实际引脚。

推荐先看：

1. `docs/hardware_connections.md`
2. `docs/serial_protocol.md`
3. `firmware/mspm0/README.md`
4. `k230/README.md`
