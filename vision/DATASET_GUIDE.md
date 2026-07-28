# 如何构建 10mm Q235 钢球数据集

## 1. 采集图片

优先用 K230 实际摄像头采集，因为视角、畸变、曝光和比赛时最接近。手机可以补样本，但不要只靠手机图。

第一轮建议先做到 700 张左右：

| 场景 | 数量 |
|---|---:|
| 单个钢球，近中远距离 | 180 |
| 画面边缘、角落、远距离小球 | 140 |
| 同时出现 2 到 5 个钢球 | 160 |
| 强反光、阴影、杂乱背景 | 120 |
| 无球负样本 | 100 |

10mm 球很小，远距离图一定要拍；否则模型只会认近处大球。

## 2. 标注

推荐工具：

- LabelImg
- Roboflow
- CVAT
- Label Studio

导出格式选择 YOLO。类别只建一个：

```text
steel_ball
```

标注规则：

- 一张图里看到几个钢球就标几个，不能只标最明显的那个。
- 框贴住球的外轮廓，宁可略紧，不要把大片背景框进去。
- 反光点、影子、螺丝、圆孔、圆形贴纸都不要标成球。
- 半遮挡但人能判断是钢球的，也要标。
- 无球图片保留，label 文件为空。

整理成原始目录：

```text
raw_ball_dataset/
  images/
    000001.jpg
    000002.jpg
  labels/
    000001.txt
    000002.txt
```

## 3. 划分训练集

```bash
python vision/scripts/split_yolo_dataset.py --raw raw_ball_dataset --out vision/dataset --val-ratio 0.2
```

检查标注：

```bash
python vision/scripts/check_yolo_labels.py --labels vision/dataset/labels
```

## 4. 训练

```bash
python -m pip install -r vision/requirements.txt
python vision/scripts/train_yolo.py --data vision/dataset/data.yaml --imgsz 416 --epochs 160
```

训练结束后会保存：

```text
vision/models/steel_ball_yolo11n.pt
```

## 5. 导出

```bash
python vision/scripts/export_onnx.py --weights vision/models/steel_ball_yolo11n.pt --imgsz 416
```

得到：

```text
vision/exports/steel_ball_yolo11n.onnx
```

## 6. 量化校准图片

从训练图片中挑 50 到 200 张代表性图片，复制到：

```text
vision/calib_images/
```

不要只放清晰好看的图，要包含暗光、反光、远距离、小球、多球和无球背景。

## 7. 判断模型是否够用

先看验证集：

```text
mAP50 > 0.90
漏检少于 5%
误检少于 5%
```

再看实车画面：

```text
连续 10 帧中至少 8 帧能检出
多个球时每个可见球都能出框
球在画面边缘也能检出
无球时不乱报
```

如果漏检多：补远距离、小球、暗光、多球样本。

如果误检多：补无球负样本、反光背景、圆形干扰物。
