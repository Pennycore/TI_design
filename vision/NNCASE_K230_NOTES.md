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
/sdcard/examples/kmodel/steel_ball_yolo11n.kmodel
```

再运行：

```text
k230/yolo_ball_detect_canmv.py
```

`k230/yolo_ball_detect_canmv.py` 已按 SD 卡中的 `PipeLine + AIBase + aidemo` 示例完成适配，包含 YOLO 后处理、主目标选择、多球结果和 UART2 数据发送。模型输入必须保持 `416x416`，与 ONNX/KModel 转换尺寸一致。

## 本机转换命令

当前项目使用 `k230` Conda 环境中的 `nncase 2.11.0` 和 `nncase-kpu 2.11.0`。项目内的 `.tools/dotnet` 提供 nncase 所需的 .NET 7 Runtime。

在项目根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File vision\scripts\convert_k230_kmodel.ps1 --samples 100
```

输出文件：

```text
vision/exports/steel_ball_yolo11n.kmodel
```

转换脚本默认从 `vision/dataset/images/train` 均匀选取 100 张图片，使用 `uint8/uint8 + KLD` 做 PTQ，并采用与板端一致的 `416x416` RGB 输入。
