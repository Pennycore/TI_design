# TI_design 工程审查资料

> 本文件由只读整理任务生成：未修改任何现有 `.c/.h/.py/.syscfg` 文件，
> 未烧录，未驱动任何电机。所有结论均来自当前工作区实际代码与构建产物。

## 1. 工程快照

| 项目 | 值 |
| --- | --- |
| 工程根目录 | `C:\Users\28457\Desktop\TI_design` |
| 当前 Git 分支 | `main` |
| 当前提交 | `463bed9`（Merge branch 'main' of https://github.com/Pennycore/TI_design） |
| 工作区是否干净 | 否（无已跟踪文件被修改；存在未跟踪文件，见下） |
| 未提交文件 | `AGENTS.md`、`docs/agent_handoff.md`、`docs/project_structure.md`、`review_bundle/`（本次新生成） |
| 当前 CCS 主工程 | 是，`car_control/`（含 `car_control.syscfg`、`.ccsproject`、`targetConfigs/MSPM0G3507.ccxml`） |
| 当前比赛任务编号 | `COMPETITION_TASK_MODE = 2`（任务 2：快速跑一圈、终点 A 停车，不带球） |
| 当前 K230 主视觉脚本 | `k230/rod_ball_blob_canmv.py`（ROI + LAB/blob + alpha-beta 跟踪 + UART 0x12 帧） |

## 2. 实际参与编译的主文件

依据 `car_control/Debug/makefile`、`Debug/subdir_vars.mk`（2026-07-27 最后一次
记录的构建）和 `.cproject/.ccsproject`：

- 当前真正使用的 `main.c` 是 `car_control/main.c`，其中只有唯一一个
  `int main(void)`（第 558 行）。
- `main copy.c.txt`、`main_distance_calibration_backup.c.txt`、
  `main_line_control_backup.txt` 都是 `.txt`，不参与编译。
- 编译范围内无重复 `main()`。`firmware/mspm0/src/main.c` 和
  `firmware/mspm0/tests/test_line_and_protocol.c` 各自有 `main()`，但属于
  另一独立目录，未出现在 `car_control/Debug` 的 C_SRCS 中；当前没有证据
  表明 CCS 工程会导入 `firmware/mspm0/`。
- 存在旧实现：`system_control.c/.h` 是一套完整的 A↔B 往返状态机，但
  `main.c` 既不包含也不调用它（`main.c` 自带 `TaskState_t` 状态机）。
  若 CCS 把工程目录内所有 `.c` 自动加入编译，它会编译但完全不被使用。
- 关键风险：现有 `Debug/makefile` 的 C_SRCS 只有 `ti_msp_dl_config.c`、
  `startup_mspm0g350x_ticlang.c`、`encoder.c`、`gray_sensor.c`、
  `k230_uart.c`、`line_control.c`、`main.c`、`motor.c`、`pid.c`、
  `speed_control.c`；**缺少 `oled.c`、`stepper_motor.c`、
  `stepper_encoder.c`、`system_control.c`、`track_position.c`**。
  `car_control.map` 中 `OLED_Init`、`StepperMotor_Init`、
  `StepperEncoder_Init`、`TrackPosition_Update` 均为 UNDEFED，Debug 目录
  没有 `.out/.hex` —— 该目录记录的是 2026-07-27 的一次失败链接。
  而 `main.c`、`line_control.c`、`speed_control.c`、
  `competition_config.h` 等在 2026-07-31 14:57 仍有修改，当前源码从未在
  这个 Debug 目录中成功构建。**必须在 CCS 中 Clean Project + Build Project
  重新生成 makefile 与 SysConfig 产物后才能确认当前树可编译。**

## 3. main.c 执行流程

实际顺序（`car_control/main.c`，`MOTOR_ENCODER_TEST_ENABLE=0`、
`SPEED_CONTROL_TEST_ENABLE=0` 的正常分支）：

```text
上电
-> SYSCFG_DL_init()
-> StepperMotor_Init()
-> StepperEncoder_Init()
-> Motor_Init() -> Encoder_Init() -> GraySensor_Init()
-> K230Uart_Init() -> SpeedControl_Init() -> LineControl_Init()
-> TrackPosition_Init() -> OLED_Init()
-> g_taskState = TASK_STATE_RUNNING; OLED 显示 READY / START IN 3S
-> delay_cycles(CPUCLK_FREQ * 3U)（阻塞 3 秒启动延时）
-> Encoder_Reset(); TrackPosition_Reset(); LineControl_SetBaseSpeed(18.0)
-> 主循环：
   -> K230Uart_Poll()（循环首尾各一次）
   -> 以 SpeedControl_GetTickCount() 的 10 ms 硬件节拍为准
   -> RUNNING/FINAL_APPROACH：
      LineControl_Update()（灰度读 + 循迹 PID + 目标速度）
      TrackPosition_Update(wide_marker, 10ms)
      SpeedControl_GetStatus() + 弯道调试统计
      丢线 >=150 周期(1.5s) -> ERROR；宽线确认 -> FINAL_APPROACH
      接近终点 400mm 降速；宽线横穿时左右 3.0 匀速
   -> FINAL_APPROACH：按剩余距离 4/3/3 降速；
      剩余 <=10mm 时 LineControl_Stop() -> STOPPING
   -> STOPPING：SpeedControl_Stop()；左右轮实际速度连续 5 个周期为 0
      -> FINISHED（OLED 显示 FINISH TIME/DIST）
   -> FINISHED/ERROR：SpeedControl_Stop()，保持停车
   -> NormalMode_UpdateStatus() / Task_UpdateDebug()
   -> K230Uart_Poll(); StepperEncoder_Update()
```

要点：

- 初始化顺序：SysConfig 之后先初始化步进相关（仅停止/复位，不产生运动），
  再初始化小车模块；`OLED_Init()` 在最后。
- 主循环周期：循迹/里程/计时由 `TIMER_CONTROL`（TIMA1，10 ms 中断）的节拍
  驱动；`SpeedControl_Update()` 在中断内运行。主循环其余时间快速自旋并
  轮询 UART，不 sleep。
- 阻塞延时：上电 3 秒启动延时是阻塞的；`OLED_Init()` 内含约 100 ms 延时和
  每次 I2C 写入最多 10 ms 的超时；灰度读是约 56 µs 的位操作。运行中主循环
  无长阻塞。
- 车辆开始运动：上电 3 秒后进入主循环，第一个 10 ms 节拍执行
  `LineControl_Update()` 给出左右目标速度，速度环开始输出占空比。
- 车辆停止：终点横线确认后经 FINAL_APPROACH，剩余 <=10 mm 发出停止命令，
  左右轮连续 50 ms 零速后进入 FINISHED；或丢线 1.5 s 进入 ERROR 停车。
- 当前是否调用步进电机：**没有任何运动调用**。全工程只调用
  `StepperMotor_Init()`（main.c:565）与 `StepperEncoder_Init/Update()`，
  不存在 `StepperMotor_StartSteps/StartContinuous/Stop/SetEnabled` 调用。
- 钢球闭环：**不存在**。`K230Uart_Poll()` 只收帧更新全局状态，
  `rod_ball_valid` 等数据既未参与控制，也未在 OLED 上显示。
- UART 数据用途：只被读取并保存在 `g_k230_uart_status`，供 CCS Watch
  观察；不参与循迹或任何执行器决策。

## 4. 比赛状态机

`main.c` 实际使用 `TaskState_t`（TASK_STATE_RUNNING/FINAL_APPROACH/STOPPING/
FINISHED/ERROR）：

| 状态 | 进入条件 | 执行动作 | 退出条件 | 控制车轮 | 控制步进 |
| --- | --- | --- | --- | --- | --- |
| RUNNING | 初始化完成、3 s 延时后 | 循迹 + 里程 + 终点检测；巡航 18（接近终点 400 mm 降为 4） | 宽线确认 -> FINAL_APPROACH；丢线 1.5 s -> ERROR | 是 | 否 |
| FINAL_APPROACH | `TrackPosition_IsAtFinish()` 为真（92% 圈长后宽线连续确认 50 ms） | 继续循迹；按剩余距离 100/40/10 mm 分档 4/3/3 降速 | 剩余 <=10 mm -> STOPPING | 是 | 否 |
| STOPPING | 剩余停止距离 <=10 mm | 一次 `LineControl_Stop()`；`SpeedControl_Stop()`；继续累计里程；等左右轮零速连续 5×10 ms | 零速确认 -> FINISHED | 否 | 否 |
| FINISHED | 停车确认 | `SpeedControl_Stop()`；OLED 显示用时/里程 | 无（断电或复位） | 否 | 否 |
| ERROR | `lost_count >= 150` 周期 | `LineControl_Stop()`；OLED 显示 ERROR | 无 | 否 | 否 |

当前任务 2 实际经过：RUNNING -> FINAL_APPROACH -> STOPPING -> FINISHED
（或中途 ERROR）。`competition_config.h` 中 4/5/6 模式只切换了速度/软启动
宏，`main.c` 顶层状态机仍然是任务 2 的一圈停车逻辑，任务 4/5/6 的目标点
与结束条件尚未实现（与交接文档一致）。

`system_control.c` 中另有旧状态机（WAIT_START -> LEAVE_A -> RUN_TO_B ->
PASS_B -> RETURN_TO_A -> BRAKING -> FINISHED），`main.c` 未调用：

- 潜在问题：`RETURN_TO_A` 依赖 `b_has_been_passed`，`RUN_TO_B` 依赖 B 点
  连续 30 ms，若标记逻辑与实际赛道不符可能卡在某个状态；`LEAVE_A` 若 A 点
  不清除也有 1 s 兜底。以上均为未使用代码，不影响当前运行。
- 当前 `main.c` 状态变量均有初始化（`g_taskState = TASK_STATE_RUNNING`），
  不存在“初始化后不启动”的代码路径（3 s 后自动开始）。
- “加入球杆系统后循迹不运行”在代码层面不成立：球杆模块目前只做
  Init/Update，不会覆盖循迹速度，也不会阻塞主循环。

## 5. K230 UART 协议逐字节核对

核对对象：`k230/rod_ball_blob_canmv.py`、`car_control/k230_uart.c/.h`、
`docs/serial_protocol.md`。

实际帧：

```text
A5 5A | msg_id | seq | payload_len | payload | crc8
```

消息 `0x12`（ROD_BALL_POSITION）逐字段：

| 字段 | K230 类型 | MSPM0 类型 | 字节序 | 长度 | 是否一致 |
| --- | --- | --- | --- | --- | --- |
| 帧头 | 0xA5 0x5A | 0xA5 0x5A | - | 2 | 一致 |
| msg_id | 0x12 | MSG_ROD_BALL_POSITION 0x12 | - | 1 | 一致 |
| seq | 0..255 循环 | 记录 last_seq | - | 1 | 一致 |
| payload_len | 8 | 要求 8 | - | 1 | 一致 |
| position_mm | int16 小端 | int16 小端 | LE | 2 | 一致 |
| velocity_mm_s | int16 小端 | int16 小端 | LE | 2 | 一致 |
| raw_x | uint16 小端 | uint16 小端 | LE | 2 | 一致 |
| confidence | uint8 | uint8 | - | 1 | 一致 |
| flags | uint8 | uint8 | - | 1 | 一致 |
| crc8 | 多项式 0x07、初值 0x00 | 同左 | - | 1 | 一致 |

核对明细：

- 帧头：双方均为 `A5 5A`。
- 波特率：K230 115200，MSPM0 SysConfig 115200（IBRD 17/FBRD 23 @32 MHz），
  8N1。
- UART 编号：K230 `UART.UART2`（TX=GPIO5，RX=GPIO6）；MSPM0 `UART2`
  （RX=PB18，TX=PB17）。接线为 K230 TX -> MSPM0 RX（PB18）、K230 RX
  （GPIO6）<- MSPM0 TX（PB17），交叉正确，3.3 V TTL。
- payload 长度：K230 `struct.pack("<hhHBB")` = 8 字节；MSPM0 仅接受
  msg_id 0x12 且 len==8 时解码，一致。
- CRC 覆盖范围：K230 `crc8(header[2:] + payload)`（msg_id、seq、len、
  payload）；MSPM0 从 msg_id 开始逐字节累计，一致。多项式 0x07、初值 0。
- 小端序：K230 `<hhHBB` 与 MSPM0 `read_i16_le/read_u16_le` 一致。
- 接收状态机重新同步：MSPM0 解析器对错误 CRC、超长 payload 都会复位到
  等待 `A5`；payload 中出现伪 `A5` 时下一步要求 `5A` 失败即复位，具备
  重同步能力。
- 数据超时：**双方都没有实现**。`k230_uart.c` 无时间戳、无
  “N ms 未收到帧”判定；若 UART 断开，`rod_ball_valid` 和 position/velocity
  会永久保留最后一次的值。当前 main 不使用这些数据，因此无实际现象；
  一旦接入闭环，必须增加超时失效（见第 12 节 H1）。
- predicted/coast 标志：K230 漏检 1..6 帧时发 `flags=STABLE|PREDICTED`
  （detected 不置位），漏检 >6 帧后跟踪器复位并发送全 0 payload。
- valid 判断：MSPM0 `rod_ball_valid = detected && stable`，coast 帧
  detected=0，不会被当作有效测量；预测数据不会被标记为 valid。

结论：**完全一致**（双方逐字节一致，可以直接联调接收）；唯一风险是
接收端缺少超时失效机制，属于后续控制层必须补齐的安全项，不属于协议
不一致。

## 6. K230 视觉输出

依据 `k230/rod_ball_blob_canmv.py`：

- 图像分辨率：640 x 480，RGB565。
- ROI：`ROD_LEFT_X=16`、`ROD_RIGHT_X=624`、`ROD_CENTER_Y=306`、
  `DETECT_ROI_HEIGHT=44`，即横跨 608 像素、高 44 像素的窄条。
- 三点标定：`-50 mm -> x=195`、`0 mm -> x=315`、`+50 mm -> x=442`。
- 像素->毫米：以 0 mm 为界分段线性；左段每像素 50/120 mm，右段每像素
  50/127 mm。速度换算使用同一分段比例。
- 位置正方向：画面 x 增大（向右）为 +mm，左负右正，与文档一致。
- 速度正方向：与位置一致（向右为正）。
- 发送频率：每处理完一帧发一帧，无节流；近期台架约 47-48 FPS
  （约 21 ms/帧），seq 0..255 循环。
- 检测状态 `searching`：跟踪器未初始化且无确认目标。
- `acquire`：`AcquireLatch` 要求同一位置附近（28 px 内）连续 3 次命中，
  跟踪器已初始化但未 locked。
- `track`：跟踪器连续命中 >=3 次后 `locked`，输出测量+滤波位置。
- `coast/predicted`：漏检 1..6 帧，用恒速外推，置
  `PREDICTED(0x08)`；漏检 >6 帧复位为 searching。
- 丢球时发送的数据：`track is None` 时全 0（flags=0）；coast 时发送预测
  的 position/velocity/raw_x、confidence=0、flags=0x02|0x08。
- `stable` 判断：跟踪器 `locked`（连续命中 3 次）。
- `on-target` 判断：`ball 存在 && locked && |position_mm - TARGET_MM| <= 10`，
  其中 **`TARGET_MM = 0` 是脚本内硬编码**（仅对“停在中心”有效；
  任务 6 的任意初始目标捕获尚未实现）。
- 预测数据是否会被标记 valid：不会。MSPM0 要求 detected+stable，coast 帧
  不含 detected。
- int16 溢出：发送前做了 clamp（position/velocity ±32767、raw_x 0..65535、
  confidence 0..100），实际量程远小于边界，不会溢出。
- RTSP：本脚本不含（打印 “WiFi is not enabled in this detector test”）。
- 显示操作：每帧执行 `draw_overlay`（画 ROI、标定线、状态文字）并
  `Display.show_image(img)` 到 IDE 虚拟显示，另每 15 帧打印调试行；
  `DRAW_DEBUG_BLOBS=False`。这些会占用部分帧时间，台架实测约 47-48 FPS，
  属可接受但需装车后复测。
- **装车后必须重新测量**：`ROD_LEFT_X`、`ROD_RIGHT_X`、`ROD_CENTER_Y`、
  `DETECT_ROI_HEIGHT` 以及 `CAL_X_NEG_50_MM/CAL_X_ZERO_MM/CAL_X_POS_50_MM`
  三个标定点；标定只对当前相机姿态有效。

## 7. 步进电机安全审查

依据 `stepper_motor.c/.h` 与全工程调用位置：

| 检查项 | 当前实现 | 文件及函数 | 结论 |
| --- | --- | --- | --- |
| 最大频率 | 200 Hz | `stepper_motor.c` `StepperMotor_ConfigureFrequency`（STEPPER_MAX_FREQUENCY_HZ） | 符合安全要求 |
| 单次最大步数 | 1 | `StepperMotor_Start`（STEPPER_MAX_STEPS_PER_COMMAND） | 符合安全要求 |
| 连续模式 | 恒返回 false | `StepperMotor_StartContinuous` | 已禁用 |
| 软件当前位置 | 无累计步数变量（仅有运行中剩余步数） | `g_stepperRemainingSteps` | 缺失，后续闭环前必须新增 |
| 正向限位 | 无 | - | 缺失 |
| 负向限位 | 无 | - | 缺失 |
| 中立位置 | 无标定常量 | - | 缺失（待实车测量） |
| 方向定义 | `NEGATIVE=0/POSITIVE=1`，DIR 高电平=POSITIVE | `stepper_motor.h/.c` | 仅电气定义，机械方向未测量 |
| UART 丢失停止 | 无（k230_uart.c 无超时，且无控制器调用步进） | - | 缺失，当前无实际影响 |
| 视觉丢失停止 | 无基于视觉的停止逻辑 | - | 缺失，当前无实际影响 |
| 阻塞式脉冲 | 非阻塞：定时器 IRQ 输出脉冲，StartSteps 立即返回 | `PWM_STEPPER_INST_IRQHandler` | 不阻塞主循环 |
| 是否影响循迹周期 | 当前无任何运动调用 | - | 不影响 |

`car_control/` 全部步进接口调用清单：

| 调用文件 | 调用函数 | 传入参数 | 是否可能绕过安全限制 |
| --- | --- | --- | --- |
| `main.c:565` | `StepperMotor_Init()` | 无 | 否（只复位/停表） |
| `main.c:566` | `StepperEncoder_Init()` | 无 | 否（编码器，不产生运动） |
| `main.c:813` | `StepperEncoder_Update()` | 无 | 否 |

**当前没有任何 `StepperMotor_StartSteps/StartContinuous/Stop/SetEnabled`
调用**，步进电机不会动作。安全限制在驱动层内，未来接入闭环时调用
`StartSteps` 也会被单步/200 Hz/禁连续限制拦截；但位置限位与 UART/视觉
丢失停机需要在控制器层补。

## 8. stepper_encoder 的真实用途

依据 `stepper_encoder.c/.h`、`car_control.syscfg`、
`car_control/HARDWARE_INTERFACES.md`：

- 它连接的是 **MS42CG 步进电机自带编码器接口**（六芯插座
  GND/Z/PWM/B/A/VCC），不是外置角位移传感器：
  - A -> PA1 / TIMG8_CCP0（QEI_STEPPER）
  - B -> PA0 / TIMG8_CCP1
  - PWM（绝对角 PWM 输出）-> PB20 / TIMG12_CCP0（CAPTURE_STEPPER_PWM，
    COMBINED 脉宽/周期捕获，60 ms 周期）
  - Z -> PA25 GPIO 输入（下拉）
- SysConfig 已配置对应外设与引脚（TIMG8、TIMG12、PA25），与代码宏一致。
- `main.c` 调用 `StepperEncoder_Init()`（在 `Encoder_Init()` 之前）和每
  10 ms 周期调用 `StepperEncoder_Update()`；但**没有任何控制逻辑读取**
  `GetCount/GetAngleDegrees/GetIndexCount/GetPwmAngleDegrees`。
- 当前控制不依赖它：`GetPwmAngleDegrees` 返回 false（未捕获到有效 PWM）
  也不会卡住；QEI 无信号时计数恒 0。硬件缺失不会导致程序卡死。
- 与交接文档“角位移传感器不可用、不得依赖”的关系：存在文档冲突。
  `HARDWARE_INTERFACES.md` 把 MS42CG 的 QEI/PWM/Z 当作可用资源描述；
  交接文档则要求控制只用步数计数 + 相机反馈，不依赖角度传感器。
  当前代码两边都不得罪（不依赖），但**闭环前必须和团队确认 MS42CG
  编码器是否实际装好/可用**，否则将来若以它做内环会与交接结论冲突。

## 9. SysConfig 引脚和资源表

依据 `car_control.syscfg`（并核对 `Debug/ti_msp_dl_config.h`）：

| 功能 | SysConfig 实例 | 引脚 | 方向/模式 | 定时器或 UART | 代码使用位置 | 冲突情况 |
| --- | --- | --- | --- | --- | --- | --- |
| 左电机 PWM | PWM_MOTOR C0 | PA8 | 输出 | TIMA0 | `motor.c` | 无 |
| 右电机 PWM | PWM_MOTOR C1 | PA9 | 输出 | TIMA0 | `motor.c` | 无（注意 J14 需选 PA9，见 MIGRATION_NOTES） |
| TB6612 AIN1 | GPIO_MOTOR_DIR | PB4 | 输出 | GPIO | `motor.c` | 无 |
| TB6612 AIN2 | GPIO_MOTOR_DIR | PB0 | 输出 | GPIO | `motor.c` | 无 |
| TB6612 BIN1 | GPIO_MOTOR_DIR | PB12 | 输出 | GPIO | `motor.c` | 无 |
| TB6612 BIN2 | GPIO_MOTOR_DIR | PB13 | 输出 | GPIO | `motor.c` | 无 |
| TB6612 STBY | GPIO_MOTOR_STBY | PB3 | 输出 | GPIO | `motor.c` | 无 |
| 左编码器 A/B | GPIO_ENCODER | PB7/PB6 | 输入上拉，双边沿中断 | GPIO GROUP1 | `encoder.c` | 无 |
| 右编码器 A/B | GPIO_ENCODER | PB9/PB8 | 输入上拉，双边沿中断 | GPIO GROUP1 | `encoder.c` | 无 |
| 灰度 CLK | GPIO_GRAY | PB15 | 推挽输出 | GPIO | `gray_sensor.c` | 无 |
| 灰度 DAT | GPIO_GRAY | PB2 | 输入上拉 | GPIO | `gray_sensor.c` | **文档冲突**：hardware_connections.md / mspm0_pin_plan.md 写 PB14，syscfg 与 HARDWARE_INTERFACES.md 为 PB2 |
| K230 UART RX | UART_K230 | PB18 | UART2 RX，115200 | UART2 | `k230_uart.c` | 无 |
| K230 UART TX | UART_K230 | PB17 | UART2 TX | UART2 | （暂未发送） | 无 |
| 步进 STEP | PWM_STEPPER C0 | PA23 | 输出 | TIMG7_CCP0，预分频 80 | `stepper_motor.c` | 无（PA23 兼 VREF+，文档已警告勿外接 VREF+） |
| 步进 DIR | GPIO_STEPPER | PA13 | 输出 | GPIO | `stepper_motor.c` | 无 |
| 步进 EN | GPIO_STEPPER | PA12 | 输出，初始低 | GPIO | `stepper_motor.c` | 无 |
| 步进编码器 A/B | QEI_STEPPER | PA1/PA0 | 输入 | TIMG8 QEI | `stepper_encoder.c` | 无 |
| 步进 PWM 角捕获 | CAPTURE_STEPPER_PWM | PB20 | 输入捕获 | TIMG12，60 ms | `stepper_encoder.c` | 无 |
| 步进编码器 Z | GPIO_STEPPER_Z | PA25 | 输入下拉 | GPIO | `stepper_encoder.c` | 无 |
| OLED | I2C_OLED | PA10(SDA)/PA11(SCL) | I2C0 主，400 kHz | I2C0 | `oled.c` | 潜在：PA10 在 LaunchPad 上经 J21 接 XDS110 UART 通道，需确认 J21 状态（见第 11/12 节） |
| 板载 LED | 无 | - | - | - | - | 未配置 |
| 调试口 | Board/DEBUGSS | PA20(SWCLK)/PA19(SWDIO) | - | DEBUGSS | - | 无 |
| 控制节拍 | TIMER_CONTROL | - | 10 ms 周期中断 | TIMA1 | `speed_control.c` | 无 |

重点检查结论：

- SysConfig 内**没有发现一个引脚被多个实例占用**；UART2（PB17/PB18）、
  TIMG7、TIMG8、TIMG12、TIMA0、TIMA1、I2C0 互不重复。
- 定时器与 PWM 通道无重复：TIMA0（电机 C0/C1）、TIMA1（控制节拍）、
  TIMG7（步进 STEP）、TIMG8（QEI）、TIMG12（PWM 捕获）各司其职。
- 步进代码宏与生成名称一致（`GPIO_STEPPER_DIR_PIN/EN_PIN`、
  `PWM_STEPPER_INST`、`QEI_STEPPER_INST`、`CAPTURE_STEPPER_PWM_INST`、
  `GPIO_STEPPER_Z_Z_PIN` 均匹配）。
- 主要问题在**文档与 syscfg 不一致**：灰度 DAT（PB14 vs PB2）、
  “K230 UART 还没合入”（hardware_connections.md 旧文，实际 syscfg 已有
  UART2），以及 OLED SDA 占用 PA10 后 J21 调试串口通道的潜在冲突。

## 10. 循迹控制完整性

依据 `line_control.c/.h`、`gray_sensor.c`、`speed_control.c`、`encoder.c`：

- 灰度数据位序：`GraySensor_Read()` 返回 bit0..bit7 = 探头 1..8；白=1、
  黑=0（模块校准后）。`LINE_CONTROL_TRACK_BLACK_LINE=1` 取反得到 line_bits
  （黑线=1）。
- 黑白逻辑：`LINE_CONTROL_BIT0_IS_LEFT=1`，权重表 `-3900..+3900` 从物理
  左到右排列，位置 = 命中探头加权平均 / 3900，归一化 -1（左）..+1（右）。
- `GraySensor_Read()` 耗时：8 通道位操作，每通道约 2 µs 低 + 5 µs 高，
  合计约 56 µs，远小于 10 ms 周期。
- 循迹周期：10 ms，由 TIMA1 节拍驱动，`LineControl_Update()` 每次只跑一次。
- 误差定义：`position = 加权质心/3900`，正值表示线在右侧；
  `correction = PID(position, 0) + 1.2*position*|position|`，限幅 ±4.8。
- 左右轮修正方向：正 position（线偏右）时 `left = base + corr`、
  `right = base - corr`（左轮加速、右轮减速），与实车方向测试一致；
  `LINE_CONTROL_SWAP_MOTOR_CHANNELS=0` 不交换通道。
- 目标速度：任务 2 巡航 18 计数/10 ms；曲线减速系数 5.0；软启动关闭
  （任务 2 加速步长 100，近似瞬达）；最大轮速 22，最小 base 2.2；
  起步低速时转向修正按 0.5*base 限幅。
- 编码器正负号：`ENCODER_LEFT_DIRECTION=-1`、`ENCODER_RIGHT_DIRECTION=1`，
  前进时左右计数应为正。
- 速度闭环调用顺序：TIMA1 中断内 `Encoder_Sample()` -> `SpeedControl_Update()`
  -> `Motor_SetBoth()`；主循环只设置目标。
- 失线处理：丢线先保持上次目标 2 帧；再按最后方向搜索最多 60 帧
  （一侧 4.0、另一侧 0）；超过 60 帧停车；main 在丢线 150 帧（1.5 s）进
  ERROR。**不会反转车轮**（目标限 0..22）。
- 直角转弯处理：无专门直角/原地转向状态机；弯道靠曲线减速 + 非线性转向
  补偿；宽线横穿时左右各 3.0 匀速直行跨过。
- 状态机是否覆盖循迹速度：是，main 在接近终点时逐档降速、宽线时置
  等速 3.0、停车/故障时 `SpeedControl_Stop()`，属设计行为。
- 步进脉冲是否阻塞循迹：当前没有任何步进运动调用；驱动本身用定时器
  IRQ，非阻塞，接入后单脉冲（200 Hz）也不会阻塞 10 ms 循迹。

## 11. OLED 影响检查

依据 `oled.c`：

- 初始化失败不会阻塞：`OLED_WaitIdle()` 有约 10 ms 超时；每次写失败后
  置 `g_oledWriteFailed` 并退出，`OLED_Init()` 最坏约 100 ms 上电延时 +
  26 次写 × 10 ms 超时后返回，`g_oledReady=0`。
- 引脚：I2C0 SDA=PA10、SCL=PA11（400 kHz），与 SysConfig 其它外设无重复；
  但 PA10 在 LaunchPad 上可能经 J21 与 XDS110 UART 通道相连，
  `MIGRATION_NOTES.md` 提到过该排针的干扰问题（当时针对 STBY），现在
  PA10 被 OLED SDA 占用，**需要确认 J21 是否断开**，否则 OLED 可能异常。
- OLED 更新频率：只在状态切换时调用（READY/RUN/FINISH/ERROR），主循环
  每 10 ms 不刷新 OLED；`OLED_Show*` 前都有 `OLED_IsReady()` 保护。
- 长延时：仅初始化时有约 100 ms 上电等待 + 每次 I2C 超时 10 ms；运行中
  无。
- OLED 不接时：`OLED_Init()` 超时后 ready=0，所有显示函数立即返回，
  整车照常运行。**“加入球杆系统后小车不动”不能由 OLED 代码解释**；
  更可能的原因是烧录/构建问题或硬件接线（见第 12 节）。

## 12. 风险分级

### 致命问题

**F1. 当前工程构建/烧录状态无法验证，现有 Debug 产物是失败的旧链接**

```text
文件：car_control/Debug/makefile、Debug/subdir_vars.mk、Debug/car_control.map
函数或位置：makefile C_SRCS / 链接器
实际代码行为：C_SRCS 缺 oled.c、stepper_motor.c、stepper_encoder.c、
system_control.c、track_position.c；map 中 OLED_Init、StepperMotor_Init、
StepperEncoder_Init、TrackPosition_Update 为 UNDEFED；Debug 下无 .out/.hex；
Debug 目录时间 2026-07-27，而 main.c 等源码 2026-07-31 14:57 仍被修改。
可能现象：按现有产物无法烧录；在 CCS 里直接 Build（不 Clean）可能
报未定义符号或继续用过期 SysConfig 头（灰度宏 AD0/AD1/AD2 vs CLK/DAT）。
推荐后续处理：在 CCS 中 Clean Project + Build Project，确认 0 error、
生成 .out 后再进入任何硬件测试。
```

### 高风险问题

**H1. K230 UART 无超时失效，旧数据会永久保留**

```text
文件：car_control/k230_uart.c（K230Uart_Poll/decode_rod_ball_payload）
函数或位置：rod_ball_valid 计算处
实际代码行为：帧到达才更新状态；没有时间戳/超时，UART 断开后
rod_ball_valid、position、velocity 保持最后值不变。
可能现象：将来接入球杆闭环后，K230 掉线/线缆松动时控制器仍认为视觉
有效，可能继续输出步进指令。
推荐后续处理：在 MSPM0 侧加“最近收帧节拍”与超时（如 200 ms 无帧即
清 valid 并进入 LOST/FAULT 停止状态）；当前 main 未使用，暂无现象。
```

**H2. 步进电机没有任何软件位置与限位**

```text
文件：car_control/stepper_motor.c
函数或位置：StepperMotor_Start / g_stepperRemainingSteps
实际代码行为：不累计已执行步数，无正负限位、无中立点标定常量；
当前无调用，所以不会动作。
可能现象：未来接入闭环后，若直接按误差发脉冲，可能把脆弱连杆推到
机械极限之外造成损坏。
推荐后续处理：先按交接流程实车测量中立点与双向安全行程，再在驱动层
加 step_count 累计与硬限位；未完成前不要调用 StartSteps。
```

**H3. 灰度 DAT 引脚文档与 SysConfig 不一致（PB14 vs PB2）**

```text
文件：docs/hardware_connections.md、docs/mspm0_pin_plan.md
函数或位置：灰度传感器接线表
实际代码行为：文档写 DAT=PB14；car_control.syscfg 与
HARDWARE_INTERFACES.md 实际为 PB2（MIGRATION_NOTES 说明已迁移）。
可能现象：按旧文档接线，灰度读不到数据，循迹丢失 1.5 s 后 ERROR，
小车不动或失控。
推荐后续处理：以 syscfg（PB2）为准接线，并更新两份旧文档。
```

**H4. K230 on_target 硬编码为 0 mm（仅中心有效）**

```text
文件：k230/rod_ball_blob_canmv.py（TARGET_MM=0、rod_position_payload）
函数或位置：ROD_FLAG_ON_TARGET 置位条件
实际代码行为：|position_mm - 0| <= 10 才置 on_target。
可能现象：任务 6 若要求球停在裁判指定位置，on_target 标志与实际目标
不符；当前任务 2 不依赖该标志。
推荐后续处理：把任务 6 目标捕获放到 MSPM0（连续 detected+stable 后
取初始均值），或由 MSPM0 下发目标；不要硬编码中心。
```

**H5. OLED SDA(PA10) 与 LaunchPad J21/XDS110 UART 通道可能冲突**

```text
文件：car_control.syscfg（I2C_OLED）、car_control/MIGRATION_NOTES.md
函数或位置：I2C0 SDA=PA10
实际代码行为：PA10 曾因 J21 与 XDS110 UART 相连被弃用（STBY 改 PB3），
现在又被用作 OLED SDA。
可能现象：若 J21 仍保持连接，I2C 通信可能被调试串口负载干扰，OLED
初始化失败（有超时，不会卡死整车）。
推荐后续处理：检查/断开 J21，或把 OLED 移到其它空闲 I2C 引脚。
```

### 一般问题

**G1. system_control.c 是未使用的第二套状态机**

```text
文件：car_control/system_control.c/.h
函数或位置：SystemControl_Update
实际代码行为：main.c 未包含/调用；其 A↔B 逻辑与 main 的 TaskState_t
并存。
可能现象：后续维护者误改其中一套，或以为它生效，造成状态机行为与
预期不符。
推荐后续处理：闭环集成前明确删除或归档该系统文件，避免双实现。
```

**G2. serial_protocol.md 含过期策略描述**

```text
文件：docs/serial_protocol.md
函数或位置：0x10/0x11/0x20 章节及“MSPM0 当前策略”
实际代码行为：当前 K230 只发 0x12；MSPM0 只解析 0x10/0x12；
无 0x11 解码、无 0x20 遥测发送；文档中“检测到即视觉对准”策略不存在。
可能现象：审查者按文档推断出未实现的视觉对准/多球行为。
推荐后续处理：以代码为准更新文档，注明 0x10/0x11/0x20 为预留。
```

**G3. Debug/ti_msp_dl_config.h 与当前 syscfg 命名不一致**

```text
文件：car_control/Debug/ti_msp_dl_config.h（过期生成物）
函数或位置：GPIO_GRAY_AD0/AD1/AD2
实际代码行为：旧头文件含 AD0/AD1/AD2 三个灰度引脚（含 PB16），当前
syscfg 为 CLK/DAT 两个；gray_sensor.c 需要 CLK/DAT 宏。
可能现象：不做 Clean 的增量构建报“未定义 GPIO_GRAY_CLK_PIN”。
推荐后续处理：执行 Clean Project + Build Project，以重新生成的
ti_msp_dl_config.h 为准。
```

**G4. K230 每帧显示/绘制开销**

```text
文件：k230/rod_ball_blob_canmv.py（draw_overlay、Display.show_image、print）
函数或位置：main() 主循环
实际代码行为：每帧画 ROI/标定线/文字并推送 IDE 显示，每 15 帧打印。
可能现象：显示占用帧时间，装车后 FPS 可能下降。
推荐后续处理：装车复测 FPS；必要时仅保留 UART 输出并关闭显示。
```

**G5. 测试宏若误开会导致上电驱动电机**

```text
文件：car_control/main.c（MOTOR_ENCODER_TEST_ENABLE /
SPEED_CONTROL_TEST_ENABLE）
函数或位置：main() 测试分支
实际代码行为：两个宏当前均为 0；若改为 1，上电 3 s 后会直接输出
40% 占空比（测试分支注释要求抬轮）。
可能现象：实车误刷后车轮突然转动。
推荐后续处理：烧录前检查两个宏为 0；测试时严格抬轮并断电机电源。
```

**G6. 里程按绝对增量累计，倒车也计为正里程**

```text
文件：car_control/track_position.c（TrackPosition_AbsoluteDelta）
函数或位置：TrackPosition_Update
实际代码行为：左右轮计数增量取绝对值再折算距离。
可能现象：停车滑行/被推回时里程偏大，终点定位提前（注释为有意取舍）。
推荐后续处理：保持现状，实车验证停车精度后再决定是否改为有符号积分。
```

**G7. K230Uart_Poll 主循环首尾重复调用**

```text
文件：car_control/main.c
函数或位置：while(1) 首尾
实际代码行为：同一周期内轮询两次 UART。
可能现象：无功能错误，仅冗余；帧计数统计正常。
推荐后续处理：保留一处即可（低优先级）。
```

## 13. CCS Expressions 调试变量建议

优先使用已存在的全局变量（均为 volatile，可在 Watch 中直接观察）：

### 系统状态

- `g_taskState`（0=RUNNING,1=FINAL_APPROACH,2=STOPPING,3=FINISHED,4=ERROR）
- `g_taskElapsedTimeMs`、`g_taskDistanceMm`、`g_taskRemainingToStopMm`
- `g_taskFinishMarkerConfirmed`、`g_competitionTaskMode`

### 灰度与循迹

- `g_lineControlLive.raw_sensor` / `line_bits` / `active_count`
- `g_lineControlLive.line_detected` / `wide_marker` / `lost_count` /
  `search_active`
- `g_lineControlLive.position` / `correction` / `running_base` /
  `left_target` / `right_target`
- `g_curve1Debug` / `g_curve2Debug` / `g_curveDebugActiveSegment`

### 左右轮编码器与速度

- `g_speedControlLive.left_target` / `right_target` / `left_actual` /
  `right_actual` / `left_output` / `right_output`
- `encoder.c` 的 `g_leftCount/g_rightCount/g_leftSpeed/g_rightSpeed` 为
  静态变量，CCS 可尝试添加但跨文件不可见；建议用上面的 Live 结构。

### 赛道位置

- `g_taskDistanceMm`、`g_taskRemainingToStopMm`、`g_taskFinishMarkerConfirmed`
- `track_position.c` 的 `g_track` 为静态，需断点查看。

### UART 接收

- `g_k230_uart_status.byte_count` / `frame_count` / `crc_error_count` /
  `bad_length_count` / `last_msg_id` / `last_seq`

### 钢球位置和速度

- `g_k230_uart_status.rod_ball_valid` / `rod_ball_detected` /
  `rod_ball_stable` / `rod_ball_on_target` / `rod_ball_predicted`
- `g_k230_uart_status.rod_ball.position_mm` / `velocity_mm_s` /
  `raw_x` / `confidence` / `flags`
- `g_k230_uart_status.ball`（旧 0x10 球消息，当前 K230 不发送）

### 步进电机

- `stepper_motor.c` 的 `g_stepperBusy` / `g_stepperContinuous` /
  `g_stepperRemainingSteps`（static，断点可见；当前不存在全局化版本）
- `stepper_encoder.c` 的 `g_encoderExtendedCount` / `g_encoderIndexCount` /
  `g_pwmCaptureValid` / `g_pwmPeriodTicks` / `g_pwmHighTicks`（static）
- 以下变量**当前不存在，后续可新增**：`stepper_position_steps`、
  `stepper_min_steps`/`stepper_max_steps`、`stepper_neutral_steps`、
  `ball_target_mm`、`uart_last_rx_tick`、`uart_timeout_ms`

## 14. 编译风险

完整静态检查结果见 `01_build_result.txt`。本轮**未执行** CCS Clean/Build
（机器上有 CCS 21 与 SDK，但只读整理任务不调用会重新生成 SysConfig 产物
与 makefile 的构建流程）。静态检查结论：

- 未发现缺失头文件或声明/实现不一致；编译范围内无重复 `main()`，
  无 `firmware/mspm0` 误导入迹象。
- 已知编译风险全部来自**构建清单与生成物过期**：
  1. `Debug/makefile`（2026-07-27）缺 5 个模块，上次链接因未定义符号失败；
  2. `Debug/ti_msp_dl_config.h` 为旧 syscfg 生成（灰度宏 AD0/AD1/AD2），
     与当前 `gray_sensor.c` 需要的 CLK/DAT 宏不一致；
  3. 当前源码（7/30-31 修改）从未在现有 Debug 目录成功构建过。
- 结论：必须 CCS Clean Project + Build Project 后重新验证；在生成
  `.out` 之前，任何烧录测试都不应开始。

## 15. 最终结论：推荐的下一步实物测试

**只推荐一个：最小系统烧录检查（Clean/Build + 烧录 + 确认主程序运行）。**

为什么先做它：

- 当前树自从 7/30-31 加入 OLED/步进/里程/K230 改动后，从未有过成功的
  构建记录（Debug 目录是 7/27 的失败链接）；没有可烧录的固件，UART 测试、
  循迹恢复、步进测试都无从谈起。
- 该测试不驱动任何执行器，可以同时验证“加入球杆系统后小车不动”是否
  只是构建/烧录问题。

电源与接线：

- 断开 TB6612 的 VM（电机电源）、断开 D36A 步进驱动 VIN，确保测试期间
  任何执行器都不会得电。
- 只保留：MSPM0 调试器（XDS110/SWD）、板载 3.3 V/5 V 供电、K230 可暂不
  接（或只接 GND 避免悬空）。
- OLED 可接可不接（代码对无 OLED 有超时保护）。

操作与观察：

1. CCS 中打开 `car_control`，Project > Clean Project，再 Build Project；
   确认 0 error 且生成 `.out`。
2. 烧录后观察 OLED（或 CCS Expressions）：
   - 上电先显示 READY / START IN 3S；
   - 约 3 s 后显示 RUN，`g_taskElapsedTimeMs` 开始递增；
   - 若灰度探头下方没有黑线，约 1.5 s 后进入 ERROR 是预期行为
     （证明状态机在跑）；把车放在黑线上方则保持 RUNNING。

正常结果：

- 构建 0 error、烧录成功、OLED READY->RUN、`g_taskState` 与
  `g_speedControlLive` 正常变化、车轮无任何动作（电机未供电）。

异常结果分别说明什么：

- 构建报未定义符号或 SysConfig 宏错误：构建清单/生成物未重新生成，
  确认 Clean 生效；若仍报错，则说明当前源码确有编译问题，需先修代码。
- 烧录后无 READY、`g_taskElapsedTimeMs` 不递增：复位/时钟/烧录链路问题，
  检查 SWD 与供电，与球杆系统无关。
- 卡在 READY 超过 3 s：启动延时/时钟配置问题。
- 上电立即 ERROR：灰度接线（PB2/DAT）或传感器供电问题，按 syscfg 接线
  复查，不要按旧文档 PB14 接线。

不要在本测试中安排钢球闭环或步进单脉冲测试；必须等固件确认可烧录
运行、并分别完成 UART 单向接收验证与空载单脉冲方向测量后再进入。

---

## 附录：资料包文件清单

```text
review_bundle/
|-- REVIEW_SUMMARY.md
|-- 00_git_status.txt
|-- 01_build_result.txt
`-- source_files/
    |-- car_control/（main.c、competition_config.h、car_control.syscfg、
    |   k230_uart.c/.h、stepper_motor.c/.h、stepper_encoder.c/.h、
    |   system_control.c/.h、track_position.c/.h、line_control.c/.h、
    |   speed_control.c/.h、motor.c/.h、encoder.c/.h、gray_sensor.c/.h、
    |   oled.c/.h、HARDWARE_INTERFACES.md、MIGRATION_NOTES.md）
    |-- k230/rod_ball_blob_canmv.py
    `-- docs/（agent_handoff.md、project_structure.md、serial_protocol.md、
        hardware_connections.md、mspm0_pin_plan.md）
```

请求的 33 个源文件/文档全部存在并已复制，**无缺失文件**。资料包未包含
Debug 产物、模型、数据集、图片、视频或历史备份。
