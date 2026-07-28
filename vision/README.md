# Steel Ball Detection Training

本目录是 10mm Q235 钢球识别的深度学习工程。当前推荐方案：

```text
YOLO11n 检测单类 steel_ball，输入尺寸优先 416x416，部署到 K230 kmodel
```

为什么这样选：

- 10mm 钢球在画面里通常是小目标，416 比 320 更稳；如果 K230 帧率不够，再退回 320。
- YOLO 本身支持一张图里同时检测多个球，不需要额外换算法。
- 单类别 `steel_ball` 数据集容易做，先做 500 到 1000 张就能开始迭代。
- 相比传统圆检测，YOLO 更抗反光、阴影、背景圆形物体和半遮挡。

## 目录

```text
vision/
  dataset/
    images/train/
    images/val/
    labels/train/
    labels/val/
    data.yaml
  scripts/
    split_yolo_dataset.py
    check_yolo_labels.py
    train_yolo.py
    export_onnx.py
  calib_images/
  exports/
  models/
```

## 数据集格式

使用 YOLO 标注格式，每张图片对应一个同名 `.txt`：

```text
class_id x_center y_center width height
```

数值都是相对图片宽高的 0 到 1 归一化坐标。当前只有一个类别：

```text
0 steel_ball
```

如果一张图里有多个钢球，就在同一个 label 文件里写多行，每个钢球一行。

## 训练

安装依赖：

```bash
python -m pip install -r vision/requirements.txt
```

训练：

```bash
python vision/scripts/train_yolo.py --data vision/dataset/data.yaml --imgsz 416 --epochs 160
```

导出 ONNX：

```bash
python vision/scripts/export_onnx.py --weights vision/models/steel_ball_yolo11n.pt --imgsz 416
```

如果 K230 端帧率不够，把训练和导出命令里的 `416` 改成 `320`，重新训练和转换。

## K230 部署

训练后得到：

```text
vision/exports/steel_ball_yolo11n.onnx
```

再用 K230/nncase 工具链转换为：

```text
steel_ball_yolo11n.kmodel
```

转换时需要放 50 到 200 张代表性图片到：

```text
vision/calib_images/
```

这些图片用于 PTQ 量化，必须覆盖真实比赛光照、距离、反光、小目标和无球背景。

K230 端推理脚本模板：

```text
k230/yolo_ball_detect_canmv.py
```

脚本会发送两类串口帧：

- `0x10 VISION_BALL`：主目标，一个最适合车控追踪的钢球。
- `0x11 VISION_MULTI_BALL`：最多 4 个钢球的列表，后面要做多球策略时使用。
