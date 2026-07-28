# K230 CanMV Vision

`ball_detect_canmv.py` 用于在庐山派 Lite K230D / CanMV 上运行传统圆检测小钢球识别，并通过 UART2 发送给 MSPM0。

`yolo_ball_detect_canmv.py` 是已适配当前 CanMV 固件的 YOLO11n 深度学习版本。它使用 SD 卡自带的 `PipeLine`、`AIBase`、`AI2D` 和 `aidemo`，训练和导出工程见 `../vision/`。

默认引脚按你提供的 40Pin 图：

- 物理 Pin 11 / GPIO05：UART2_TXD
- 物理 Pin 13 / GPIO06：UART2_RXD
- GND：任意 GND 引脚

如果你的 CanMV 固件或板卡示例使用的是别的 IO 编号，修改脚本顶部：

```python
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6
UART_ID = 2
```

## 调试步骤

1. 先只接 K230 的 TX、RX、GND，不接电机。
2. 将 `steel_ball_yolo11n.kmodel` 放到 `/sdcard/examples/kmodel/`。
3. 在 CanMV IDE 运行 `yolo_ball_detect_canmv.py`。
4. 用电脑串口工具或 MSPM0 遥测确认持续收到 `VISION_BALL` 帧。
5. 漏检时降低 `CONF_THRESHOLD`，误检时提高它；默认值为 `0.35`。

## 算法升级接口

后续如果换成 YOLO/kmodel，只需要保持发送同样的 `VISION_BALL` payload：`x, y, radius, offset_x, confidence, flags`。
