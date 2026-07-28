# MSPM0 与 K230 串口协议

## 串口参数

- 电平：3.3V TTL
- 波特率：115200
- 数据格式：8N1
- 默认方向：K230 发送视觉目标，MSPM0 可回传遥测

## 帧格式

```text
0      1      2       3     4      5..N       last
0xA5   0x5A   msg_id  seq   len    payload    crc8
```

CRC8 覆盖 `msg_id, seq, len, payload`，多项式 `0x07`，初值 `0x00`。

## 消息 `0x10`：VISION_BALL

K230 到 MSPM0。表示当前主目标，payload 长度 10 字节，小端序：

| 字段 | 类型 | 说明 |
|---|---|---|
| x | int16 | 球心 x，未检测到为 0 |
| y | int16 | 球心 y，未检测到为 0 |
| radius | uint16 | 半径，未检测到为 0 |
| offset_x | int16 | `x - image_width / 2` |
| confidence | uint8 | 0 到 100 |
| flags | uint8 | bit0 detected，bit1 stable，bit2 close |

MSPM0 当前策略：

- 未检测到：继续灰度循迹。
- 检测到但不稳定：低权重视觉修正或忽略。
- 稳定检测到：进入视觉对准，按 `offset_x` 差速。
- `close` 置位：停车，等待后续机构动作。

## 消息 `0x11`：VISION_MULTI_BALL

K230 到 MSPM0。表示一帧中最多 4 个钢球的检测列表。这个帧是给后续多球策略预留的，初期 MSPM0 可以先不处理。

payload：

```text
count, ball0, ball1, ...
```

其中 `count` 为 `uint8`，最大为 4。每个 ball 长度 7 字节，小端序：

| 字段 | 类型 | 说明 |
|---|---|---|
| x | int16 | 球心 x |
| y | int16 | 球心 y |
| radius | uint16 | 半径 |
| confidence | uint8 | 0 到 100 |

最大 payload 长度为 `1 + 4 * 7 = 29` 字节，小于 MCU 侧 32 字节负载限制。

## 消息 `0x20`：MCU_TELEMETRY

MSPM0 到 K230 或电脑调试，payload 长度 12 字节，小端序：

| 字段 | 类型 | 说明 |
|---|---|---|
| time_ms | uint32 | MSPM0 毫秒计时 |
| line_pos | int16 | 线位置，-3500 到 +3500 |
| left_pwm | int16 | 左轮 PWM，-1000 到 +1000 |
| right_pwm | int16 | 右轮 PWM，-1000 到 +1000 |
| state | uint8 | 状态机 |
| gray_bits | uint8 | 灰度数字量 |

## 调试建议

- 先只连 GND、TX、RX，用 `tools/serial_monitor.py` 看 K230 是否稳定发 `0x10` 或 `0x11` 帧。
- 再接电机前，先让 MSPM0 只打印 `line_pos`，用手移动灰度传感器确认左右方向正确。
- 最后接 TB6612FNG，先把基础 PWM 调到低速再试车。
