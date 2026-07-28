# K230 kmodel 转换说明

K230 通常使用 nncase 将 ONNX 转成 `.kmodel`。具体命令会随 CanMV/K230 SDK 和 nncase 版本变化，最终以你们安装的 SDK 文档为准。

通用流程：

```text
YOLO11n .pt
  -> export ONNX
  -> nncase import/compile with PTQ calib images
  -> steel_ball_yolo11n.kmodel
  -> copy to K230 SD card
```

输入建议：

```text
imgsz = 416
batch = 1
color = RGB
class = 1
```

10mm 钢球是小目标，先用 416。如果 K230 实测帧率不够，再把训练、导出、转换统一改为 320。

量化建议：

```text
PTQ int8
calib images: 50-200
calib images must match real camera scenes
```

校准图片必须包含：

- 远距离小球
- 多个钢球
- 强反光
- 暗光
- 无球背景

转换完成后，把模型放到 K230：

```text
/sdcard/models/steel_ball_yolo11n.kmodel
```

再运行：

```text
k230/yolo_ball_detect_canmv.py
```

注意：`k230/yolo_ball_detect_canmv.py` 里已经写好了串口打包和多球结果处理，但 `load_detector()` 还需要按你们实际 CanMV 固件里的 kmodel 推理 API 填上适配代码。
