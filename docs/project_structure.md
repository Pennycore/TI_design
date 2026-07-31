# 工程文件目录

更新日期：2026-07-31

本文件用于快速了解 `TI_design` 工程结构。项目当前主线是：

- K230 使用 `k230/rod_ball_blob_canmv.py` 检测钢球位置并通过 UART 发送。
- MSPM0 使用 `car_control/` 完成循迹、接收视觉数据，并负责后续步进电机闭环控制。
- `firmware/mspm0/` 是较早的可移植固件框架，不是当前 CCS 上车主工程。
- `vision/` 保存 YOLO 数据处理、训练和模型转换工具，当前不是实时控制主路径。

完整交接状态请阅读 `docs/agent_handoff.md`。

## 1. 总体目录

```text
TI_design/
|-- AGENTS.md                       # 新 agent 必读入口与硬件安全规则
|-- README.md                       # 项目总览，部分内容可能早于当前实现
|-- 1.txt                           # 早期临时记录
|-- car_control/                    # 当前 MSPM0 CCS 主工程
|-- docs/                           # 接线、协议、比赛方案和交接文档
|-- firmware/mspm0/                 # 早期跨平台固件原型及主机测试
|-- k230/                           # K230/CanMV 板端脚本
|-- tools/                          # PC 串口调试工具
|-- vision/                         # YOLO 数据集、训练、导出及 nncase 转换工具
|-- build/                          # 本地构建产物，不作为源码阅读入口
|-- runs/                           # Ultralytics 本地训练/验证输出
|-- artifacts/                      # 本地分析或导出产物
|-- Ultralytics/                    # 本地第三方/工具目录
|-- .tmp_video_review/              # 临时视频分析帧
|-- .tools/                         # 本地工具与 LabelImg 环境
|-- .codex/                         # Codex 本地配置
|-- .theia/                         # IDE 本地配置
`-- .gitignore                      # Git 忽略规则
```

`build/`、`runs/`、`artifacts/`、`.tmp_video_review/`、`.tools/` 和大部分
模型/数据集产物通常不应作为控制代码修改入口，也不应随意加入 Git。

## 2. 当前 MSPM0 主工程

```text
car_control/
|-- car_control.syscfg              # CCS SysConfig 外设、GPIO、定时器和 UART 配置
|-- main.c                          # 当前车辆主程序和比赛状态机
|-- competition_config.h            # 比赛任务模式与车速参数
|
|-- gray_sensor.c/.h                # 八路灰度传感器驱动
|-- line_control.c/.h               # 循迹误差计算与方向控制
|-- motor.c/.h                      # TB6612FNG 电机底层控制
|-- encoder.c/.h                    # 车轮编码器读取
|-- speed_control.c/.h              # 左右轮速度闭环
|-- pid.c/.h                        # 通用 PID 实现
|-- track_position.c/.h             # 赛道位置/里程相关逻辑
|-- system_control.c/.h             # 系统控制辅助逻辑
|-- oled.c/.h                       # OLED 状态显示
|
|-- k230_uart.c/.h                  # K230 二进制帧解析及钢球状态接口
|-- stepper_motor.c/.h              # 步进电机驱动，包含单步和频率安全限制
|-- stepper_encoder.c/.h            # 步进机构编码器接口
|
|-- targetConfigs/
|   |-- MSPM0G3507.ccxml            # CCS 调试目标配置
|   `-- readme.txt
|
|-- HARDWARE_INTERFACES.md          # 本工程硬件接口说明
|-- MIGRATION_NOTES.md              # 从原型迁移到 CCS 工程的记录
|-- README.md                       # car_control 子工程说明
|
|-- main copy.c.txt                 # 历史备份，不参与编译
|-- main_distance_calibration_backup.c.txt
`-- main_line_control_backup.txt
```

### 当前主线文件

新 agent 修改 MSPM0 功能时应优先阅读：

1. `car_control/main.c`
2. `car_control/competition_config.h`
3. `car_control/k230_uart.c/.h`
4. `car_control/stepper_motor.c/.h`
5. `car_control/car_control.syscfg`

所有 `.txt` 形式的 `main` 文件都是历史备份，不参与 CCS 编译。

## 3. K230 / CanMV 文件

```text
k230/
|-- rod_ball_blob_canmv.py           # 当前主线：快速 ROI + LAB/blob + 跟踪 + UART
|-- wifi_rtsp_test_canmv.py          # Wi-Fi AP 与 RTSP 图传独立测试
|-- capture_clean_video_canmv.py     # 采集无标注测试视频/困难样本
|-- yolo_ball_detect_canmv.py        # YOLO/KModel 检测实验与备用方案
|-- rod_ball_position_canmv.py       # 旧圆检测方案，帧率低，不用于当前闭环
|-- ball_detect_canmv.py             # 早期钢球检测实验
`-- README.md                        # K230 脚本使用说明，部分内容可能较旧
```

### 当前主线说明

`rod_ball_blob_canmv.py` 是目前唯一建议接入实时控制的视觉脚本。它负责：

- 在固定摇杆 ROI 中寻找钢球。
- 对候选区域进行 LAB 亮度、面积填充率、尺寸和形状筛选。
- 使用确认阶段避免锁定摇杆端点和刻度标记。
- 使用 alpha-beta 跟踪器输出平滑位置和速度。
- 将 `position_mm`、`velocity_mm_s`、原始 x、置信度和标志位通过
  K230 UART2 发送给 MSPM0。

摄像头最终装车后，必须重新标定 ROI 和 -50/0/+50 mm 三点像素坐标。

## 4. 工程文档

```text
docs/
|-- agent_handoff.md                 # 当前最完整的工程交接文档
|-- project_structure.md             # 本文件：工程目录与文件用途
|-- hardware_connections.md          # K230、MSPM0、电机和传感器接线
|-- serial_protocol.md               # A5 5A 帧、消息类型、负载和 CRC8
|-- mspm0_pin_plan.md                # MSPM0 引脚规划
|-- h_task_visual_plan.md            # H 题视觉方案与标定计划
|-- wireless_video_ipad.md           # iPad 无线图传操作记录
|-- car_control_next_steps.md        # 车辆控制阶段性操作说明
`-- algorithm_choice.md              # 早期算法选型记录
```

发生文档冲突时，优先级建议如下：

1. 实际代码和 SysConfig
2. `docs/agent_handoff.md`
3. `docs/serial_protocol.md` 与 `docs/hardware_connections.md`
4. 其他阶段性文档和旧 README

## 5. PC 串口工具

```text
tools/
`-- serial_monitor.py                # 解码 K230/MSPM0 A5 5A 二进制串口帧
```

使用示例：

```powershell
python tools\serial_monitor.py COM6 --baud 115200
```

该工具需要 USB 转串口设备才能在 PC 上监听；K230 与 MSPM0 直接连接时不
需要 USB 转串口。

## 6. 早期可移植 MSPM0 固件

```text
firmware/mspm0/
|-- include/
|   |-- board_config.h               # 原型板级配置
|   |-- gw_gray.h                    # 灰度传感器接口
|   |-- hal.h                        # 硬件抽象层
|   |-- line_control.h               # 循迹接口
|   |-- motor_control.h              # 电机接口
|   |-- robot_app.h                  # 应用状态接口
|   `-- serial_protocol.h            # 串口协议接口
|-- src/
|   |-- gw_gray.c
|   |-- line_control.c
|   |-- main.c
|   |-- motor_control.c
|   |-- robot_app.c
|   `-- serial_protocol.c
|-- port/
|   `-- ti_mspm0_port.c              # TI MSPM0 平台适配层
|-- tests/
|   |-- host_sim_port.c              # 主机模拟 HAL
|   `-- test_line_and_protocol.c      # 循迹与协议测试
`-- README.md
```

该目录用于早期模块化设计、协议验证和主机测试。真实 CCS 编译、烧录和上车
控制以 `car_control/` 为准，不要在两个实现中同时修同一个硬件问题。

## 7. YOLO 训练与模型转换

```text
vision/
|-- README.md                        # 视觉训练总说明
|-- DATASET_GUIDE.md                 # 数据集采集、标注与目录规范
|-- NNCASE_K230_NOTES.md             # ONNX 到 KModel 转换记录
|-- requirements.txt                 # PC 训练依赖
|
|-- dataset/
|   |-- data.yaml                    # YOLO 数据集配置
|   |-- images/                      # 训练/验证图片，本地大文件
|   `-- labels/                      # YOLO 标签，本地大文件
|
|-- hard_cases/                      # 困难样本、复核标签和微调数据
|-- calib_images/                    # nncase 量化校准图片
|-- models/                          # PT/ONNX/KModel 模型输出
|-- exports/                         # 导出模型
|-- runs/                            # 训练与验证结果
|-- nncase_dump/                     # nncase 转换中间文件
|
`-- scripts/
    |-- prepare_steel_ball_dataset.ps1
    |-- split_yolo_dataset.py
    |-- check_yolo_labels.py
    |-- train_yolo.py
    |-- export_onnx.py
    |-- convert_k230_kmodel.py
    |-- convert_k230_kmodel.ps1
    |-- extract_hard_case_frames.py
    |-- auto_label_hard_cases.py
    |-- prepare_hard_case_review.py
    |-- prepare_hard_case_finetune.py
    `-- finetune_hard_cases.ps1
```

### 训练链路

```text
原始图片/视频
  -> 抽帧与标注
  -> dataset/images + dataset/labels
  -> train_yolo.py
  -> .pt
  -> export_onnx.py
  -> .onnx
  -> convert_k230_kmodel.py
  -> .kmodel
  -> K230 SD 卡与 yolo_ball_detect_canmv.py
```

当前比赛控制优先使用快速 blob 跟踪。YOLO 链路应保留为困难光照/背景下的
备用方案和数据资产，不应在未测量实时帧率前直接替换主控制路径。

## 8. 本地生成内容

以下目录可能体积较大或由工具自动生成。交接时需要说明其存在，但通常不应
逐文件阅读或加入版本控制：

```text
build/                    CCS/本地构建输出
car_control/Debug/        CCS Debug 输出
runs/                     Ultralytics 根目录运行输出
vision/runs/              YOLO 训练输出
vision/models/            模型权重和导出模型
vision/exports/           ONNX/KModel 导出副本
vision/nncase_dump/       nncase 中间文件
.tmp_video_review/        视频分析临时帧
.tools/                   本地 Python/.NET/LabelImg 工具
artifacts/                临时分析产物
```

数据集和模型通常由 `.gitignore` 排除。修改忽略规则前先检查文件大小，避免把
整个训练集、训练运行目录或大模型推送到 GitHub。

## 9. 新 Agent 推荐阅读顺序

1. `AGENTS.md`
2. `docs/agent_handoff.md`
3. `docs/project_structure.md`
4. `k230/rod_ball_blob_canmv.py`
5. `docs/serial_protocol.md`
6. `docs/hardware_connections.md`
7. `car_control/k230_uart.c/.h`
8. `car_control/stepper_motor.c/.h`
9. `car_control/main.c`
10. `car_control/car_control.syscfg`

在完成上述阅读之前，不应扩大步进电机动作范围或开始整车闭环测试。
