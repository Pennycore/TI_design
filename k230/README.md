# K230 CanMV Vision

`ball_detect_canmv.py` 用于在庐山派 Lite K230D / CanMV 上运行小钢球识别，并通过 UART2 发送给 MSPM0。

默认引脚按你提供的 40Pin 图：

- 物理 Pin 11 / GPIO05：UART2_TXD
- 物理 Pin 13 / GPIO06：UART2_RXD
- GND：任意 GND 引脚

如果你的 CanMV 固件或板卡示例使用的是别的 IO 编号，修改脚本顶部：

```python
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6
UART_ID = UART.UART2
```

## 调试步骤

1. 先只接 K230 的 TX、RX、GND，不接电机。
2. 在 CanMV IDE 运行 `ball_detect_canmv.py`。
3. 用电脑串口工具或 MSPM0 遥测确认持续收到 `VISION_BALL` 帧。
4. 调整 `MIN_RADIUS`、`MAX_RADIUS`、`CIRCLE_THRESHOLD`。
5. 画面中出现误检时，缩小 `ROI` 或提高 `CIRCLE_THRESHOLD`。

## 算法升级接口

后续如果换成 YOLO/kmodel，只需要保持发送同样的 `VISION_BALL` payload：`x, y, radius, offset_x, confidence, flags`。
