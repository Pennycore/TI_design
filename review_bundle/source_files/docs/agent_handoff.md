# Agent Handoff: 2026 TI Cup H Project

Last updated: 2026-07-31

This document is the handoff entry point for the next coding agent. Read it
before editing code, changing pins, or commanding the stepper.

## 1. Snapshot

- Repository: `C:\Users\28457\Desktop\TI_design`
- Branch at handoff: `main`
- Commit at handoff: `463bed9`
- Worktree at handoff: clean; re-run `git status --short` before editing.
- Competition: 2026 TI Cup H, vehicle-mounted ball-on-beam motion control.
- Current competition mode in `car_control/competition_config.h`: task 2.

The car line-following hardware and firmware have been assembled by the team and
can follow the track. The vision algorithm works on the bench. The remaining
critical path is final mounting/calibration, K230-to-MSPM0 UART verification,
safe stepper characterization, and then closed-loop ball control.

## 2. Hardware and Responsibility Split

| Device | Current responsibility |
| --- | --- |
| LP-MSPM0G3507 | Line following, vehicle state machine, K230 UART receive, future ball controller and stepper commands |
| LCKFB Lushan K230 / CanMV | Camera capture, one-dimensional steel-ball detection/tracking, position/velocity UART output |
| Eight-channel grayscale sensor | Track following |
| TB6612FNG | Vehicle DC motor drive |
| Stepper + crank linkage | Tilts one end of the beam; mechanism is fragile |
| iPad + K230 Wi-Fi RTSP | Wireless monitoring/recording, not part of the control feedback path |

The angle displacement sensor is unusable and must not be required by the
control design. Use bounded step count as the actuator's internal reference and
camera ball position as the outer-loop feedback.

The physical mechanism has one hinged end and one stepper/crank-driven end. A
large or continuous stepper command can damage it.

## 3. Current Status

| Area | State | Notes |
| --- | --- | --- |
| Car line following | Working on hardware | `car_control/main.c` contains the line-following and finish-position state machine. |
| CCS project | Previously compiled successfully | The next agent should still Clean/Build after every integration change. |
| Fast ball vision | Bench functional | Current path is LAB/blob geometry plus acquisition/tracking at roughly 47-48 processing FPS in recent tests. |
| Edge/false-target handling | Good enough for bench, not final | Occasional misses and false endpoint/mark candidates were tuned down. User chose to finish tuning after mounting. |
| K230 UART transmit | Code ready and observed in CanMV output | Binary `0x12` frames are generated. |
| MSPM0 UART parser | Implemented | Final board-to-board receive verification on the assembled system is still required. |
| Stepper low-level driver | Implemented with strict guards | One step maximum per command; continuous mode disabled. Closed-loop ball control is not implemented. |
| Wireless video | Proven separately | iPad received RTSP video with about 1 s delay. This delay is acceptable for viewing, never for feedback control. |
| Added illumination | Forbidden | Competition officials said no added lighting. Keep auto exposure/ambient-light robustness. |

## 4. Canonical Code Map

| Path | Purpose |
| --- | --- |
| `k230/rod_ball_blob_canmv.py` | Current fast one-dimensional ball detector/tracker and UART producer |
| `k230/wifi_rtsp_test_canmv.py` | Separate Wi-Fi AP/RTSP proof of concept |
| `k230/rod_ball_position_canmv.py` | Older circle-based experiment; too slow for the control path |
| `k230/yolo_ball_detect_canmv.py` | YOLO experiment; useful as a reference/fallback, not current control path |
| `car_control/main.c` | Vehicle application and state machine; polls K230 but has no ball balancing loop yet |
| `car_control/k230_uart.c/.h` | UART frame parser and latest ball/rod-ball state |
| `car_control/stepper_motor.c/.h` | Guarded stepper interface |
| `car_control/competition_config.h` | Task mode and vehicle speed profile |
| `docs/serial_protocol.md` | UART packet format and CRC |
| `docs/hardware_connections.md` | Physical wire map |
| `tools/serial_monitor.py` | PC-side decoder for captured UART frames |

Some older README text predates the implemented `0x12` parser and fast blob
tracker. This handoff and the canonical files above take precedence when they
conflict.

## 5. Vision Implementation

### Current algorithm

`k230/rod_ball_blob_canmv.py` uses a narrow beam ROI, an adaptive LAB luminance
threshold, blob geometry filters, a confirmed acquisition stage, and a
constant-velocity alpha-beta tracker. It is intentionally one-dimensional: the
controller needs the ball's longitudinal position and velocity, not a general
object detector.

Current checked-in three-point calibration:

```text
-50 mm -> x = 195
  0 mm -> x = 315
+50 mm -> x = 442
```

The script applies piecewise pixel-to-mm scaling around zero. These values are
valid only for the camera pose in which they were measured.

Important current tuning values include:

```text
IMAGE_SIZE                 640 x 480
EXPECTED_DIAMETER          38 px
MIN_FILL_PERCENT           55
REACQUIRE_MIN_CONFIDENCE   20
TRACK_ALPHA                0.86
TRACK_BETA                 0.18
TRACK_GATE_PIXELS          140
TRACK_CONFIRM_HITS         3
MAX_TRACK_MISSES           6
ACQUIRE_CONFIRM_HITS       3
ACQUIRE_GATE_PIXELS        28
ACQUIRE_EDGE_MARGIN        18
ACQUIRE_MIN_SHORT_SIDE     12
ACQUIRE_MAX_ASPECT_PERCENT 220
```

On-screen states:

- `searching`: no confirmed target.
- `acquire`: a candidate is being confirmed.
- `track`: measured ball position is accepted.
- `coast`: short prediction through a missed detection; do not treat long coasts
  as valid control feedback.
- `cand`: candidates that passed geometry filtering.
- `acq`: whether an acquisition candidate is currently being accumulated.
- `probe`: diagnostic for a near-miss candidate, including width, height, fill,
  confidence, and readiness.

The colored vertical lines mark the calibrated -50 mm, 0 mm, and +50 mm points.
They are references, not detected objects.

### Mandatory recalibration after final mounting

1. Fix the camera rigidly. Do not continue if the mount can shift.
2. Adjust `ROD_LEFT_X`, `ROD_RIGHT_X`, `ROD_CENTER_Y`, and
   `DETECT_ROI_HEIGHT` so the ROI covers the usable beam but excludes hardware
   above and below it.
3. Place the ball at -50 mm, 0 mm, and +50 mm and record center pixel x values.
4. Update `CAL_X_NEG_50_MM`, `CAL_X_ZERO_MM`, and `CAL_X_POS_50_MM`.
5. Test an empty beam, a stationary ball at both ends and center, slow rolling,
   and fast rolling under at least two ambient lighting conditions.
6. Confirm no endpoint, red scale mark, screw, hand, or shadow stays locked as a
   ball for more than the acquisition confirmation period.

Do not add a lamp. If ambient light varies, prefer exposure/threshold robustness
and mechanical shielding from direct glare that does not illuminate the scene.

## 6. K230 to MSPM0 UART

### Wiring

| K230 physical pin | K230 signal | MSPM0 signal |
| --- | --- | --- |
| Pin 11 | GPIO5 / UART2_TX | PB18 / UART2_RX |
| Pin 13 | GPIO6 / UART2_RX | PB17 / UART2_TX |
| GND | GND | GND |

Use 3.3 V TTL and a common ground. For the first one-way receive test, connect
only K230 GPIO5 TX to MSPM0 PB18 RX plus GND. Add the reverse wire only when a
real command/acknowledgement path is needed.

### Packet format

```text
A5 5A | msg_id | seq | payload_len | payload | crc8
```

- Baud: 115200, 8N1.
- CRC-8 polynomial: `0x07`, initial value `0x00`.
- Rod-ball message ID: `0x12`.
- Payload, little endian, 8 bytes:
  - `int16 position_mm`
  - `int16 velocity_mm_s`
  - `uint16 raw_x`
  - `uint8 confidence`
  - `uint8 flags`

Flags:

```text
0x01 detected
0x02 stable
0x04 on target
0x08 predicted/coasting
```

On the MSPM0, `rod_ball_valid` currently requires both detected and stable. Use
`K230Uart_GetRodBall()` rather than parsing bytes again in the controller.

PC monitor syntax, when a USB-to-UART adapter is available:

```powershell
python tools\serial_monitor.py COM6 --baud 115200
```

The direct K230-to-MSPM0 connection does not require a PC adapter.

## 7. Stepper Safety and Control Design

Current `car_control/stepper_motor.c` guards:

```text
maximum frequency: 200 Hz
maximum command:   1 step
continuous mode:   disabled
```

Do not weaken these limits just to make the response look faster. First measure
the safe mechanism envelope with the ball removed:

1. Establish a physical neutral position.
2. Send exactly one pulse and identify which direction raises the driven end.
3. Return to neutral.
4. Count safe pulses toward each mechanical limit, stopping well before stress,
   binding, or linkage over-center conditions.
5. Encode conservative positive and negative step-count limits in software.
6. Verify a lost-vision state immediately stops new pulses.

Recommended first controller is a low-authority state machine around position
and velocity, not an unrestricted PID:

```text
SAFE_WAIT -> CAPTURE_TARGET -> HOLD -> LOST/FAULT
```

- Tasks 3-5: use the specified center/offset target.
- Task 6: after several consecutive detected+stable frames, capture the median
  or average ball position as the target. This implements "any specified
  position" without needing the operator to know the CanMV `track` text.
- Compute `error = target_mm - position_mm` and use measured
  `velocity_mm_s` for damping.
- Begin with a deadband, one pulse per decision, and a limited decision rate.
- Never issue a pulse from a predicted-only or stale measurement.
- On target loss, UART timeout, limit hit, or impossible jump, stop and enter a
  recoverable fault/lost state.

The user must be able to cut actuator power immediately during first closed-loop
tests.

## 8. Exact Next Steps

1. Mechanically mount the K230 in its final rigid position.
2. Recalibrate ROI and the three pixel/mm points; save a final screenshot and
   short test video.
3. Run the vision script alone and confirm full-beam detection under ambient
   lighting with no persistent false target.
4. Wire K230 TX -> MSPM0 RX and common GND. Keep the stepper unpowered.
5. In CCS, Clean/Build/Debug and inspect `K230Uart_GetStatus()` plus
   `K230Uart_GetRodBall()` while moving the ball by hand.
6. Confirm packet count rises, CRC errors do not rise continuously, position
   follows the ball, and timeout/lost status works when the K230 is disconnected.
7. With the ball removed, characterize one-pulse stepper direction, neutral, and
   conservative total travel limits.
8. Implement the bounded `SAFE_WAIT/CAPTURE_TARGET/HOLD/LOST/FAULT` controller.
9. Tune one pulse at a time, beginning at the beam center with the car stationary.
10. Only after stable stationary tests, run the car slowly and integrate RTSP
    monitoring/recording without allowing it to reduce control reliability.

## 9. Build and Verification

Basic host-side syntax check:

```powershell
python -m py_compile k230\rod_ball_blob_canmv.py tools\serial_monitor.py
```

K230:

1. Open `k230/rod_ball_blob_canmv.py` in CanMV IDE.
2. Connect to the LCKFB K230 board and run it.
3. Record the terminal startup line, processing FPS, state transitions, and a
   final frame screenshot.

MSPM0:

1. Open `car_control` as the CCS project.
2. Select **Project > Clean Project**.
3. Select **Project > Build Project**.
4. Debug with motors/stepper disabled for the first UART integration test.

Do not assume a successful historical build proves the current checkout and
SysConfig generation are still valid.

## 10. Known Risks and Open Work

- The camera view, ROI, and calibration will change after installation on the
  final car.
- A polished steel ball changes appearance with ambient light. Adaptive LAB
  thresholding helps but cannot replace final-condition testing.
- Endpoint hardware and scale marks have previously produced false candidates.
  The current acquisition gates reduce this, but mounted-system retesting is
  still required.
- `rod_ball_position_canmv.py` has previously fallen to roughly 3-5 FPS; do not
  confuse it with the current blob tracker.
- YOLO/KModel training and export exist as a fallback path, but the current fast
  control path is classical ROI/blob tracking.
- RTSP display has about 1 s latency and can consume camera/encoder resources.
  It is only for monitoring and required video recording.
- The MSPM0 main loop polls vision but does not yet close the loop around the
  stepper.
- No angle sensor is available, so step-count limits and a known neutral position
  are mandatory.

## 11. Questions to Resolve on the Assembled Vehicle

Before closed-loop work, obtain and record:

- Final `ROD_LEFT_X`, `ROD_RIGHT_X`, `ROD_CENTER_Y`, and ROI height.
- Final pixel x values at -50 mm, 0 mm, and +50 mm.
- Which stepper direction raises the driven end.
- Neutral step count and conservative safe counts in both directions.
- Actual MSPM0 receive statistics and timeout behavior.
- Which competition task is being tested and how its target should be captured.

## 12. Handoff Acceptance Checklist

The next agent should not call integration complete until all are true:

- [ ] Clean worktree state has been checked and user changes preserved.
- [ ] Camera is rigid and final calibration is committed/documented.
- [ ] Empty beam has no persistent false lock.
- [ ] Ball is detected over the full required travel under ambient light.
- [ ] K230-to-MSPM0 packets and timeout behavior are verified on hardware.
- [ ] Stepper direction, neutral, and conservative hard limits are measured.
- [ ] Lost vision and UART loss stop all new step commands.
- [ ] Center hold works while the car is stationary.
- [ ] Task 6 captures an arbitrary initial target correctly.
- [ ] Wireless video records and replays without compromising control.
