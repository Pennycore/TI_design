# TI Design Agent Guide

Read `docs/agent_handoff.md` before changing code or advising hardware work. It is
the current source of truth for project status, wiring, safety constraints, and
the next integration steps.

## Project Scope

This repository implements the 2026 TI Cup H problem: a line-following car with
a fragile stepper-driven ball-on-beam mechanism. The K230 measures the steel
ball position; the LP-MSPM0G3507 owns vehicle and actuator control.

## Non-Negotiable Safety Rules

- The linkage is fragile. Never increase the stepper command above one pulse per
  accepted command until direction, neutral position, and total safe travel have
  been measured on the assembled mechanism with the ball removed.
- Do not enable continuous stepper motion. The current driver intentionally
  rejects it and limits frequency to 200 Hz.
- Do not run a new closed loop on the real mechanism without hard software
  travel limits, a lost-vision stop state, and an immediately accessible power
  cutoff.
- The competition does not permit added illumination. Do not design a solution
  that depends on a lamp or LED light source.
- Treat camera position as calibration. Re-measure the ROI and the -50/0/+50 mm
  pixel points after the camera is mounted or moved.
- K230/MSPM0 UART is 3.3 V TTL. Cross TX/RX and connect GND; never apply 5 V to
  either UART pin.
- Preserve user changes in a dirty worktree. Never reset or revert unrelated
  files.

## Canonical Files

- Current fast vision implementation: `k230/rod_ball_blob_canmv.py`
- MSPM0 application: `car_control/`
- K230 UART parser: `car_control/k230_uart.c`
- Guarded stepper driver: `car_control/stepper_motor.c`
- Wire map: `docs/hardware_connections.md`
- Binary serial protocol: `docs/serial_protocol.md`
- Full project handoff: `docs/agent_handoff.md`
- Annotated repository tree: `docs/project_structure.md`

`k230/rod_ball_position_canmv.py` and the YOLO scripts are experiments and are
not the current control-path implementation.

## Required Integration Order

1. Fix the camera mechanically and recalibrate vision.
2. Validate vision alone across the complete usable beam.
3. Validate one-way K230-to-MSPM0 UART with all motors disabled.
4. Validate exactly one stepper pulse in each direction with the ball removed.
5. Measure neutral and total safe actuator travel, then add hard limits.
6. Add a low-authority ball controller and tune it from one pulse at a time.
7. Integrate wireless video only after the control path is stable.

For task 6, the target is the ball's initial valid stable position captured at
the start of the run. Do not hard-code task 6 to the beam center.

## Build Notes

- Open `car_control` in CCS, then use **Clean Project** and **Build Project**.
  Generated CCS makefiles are not the preferred manual build interface.
- Run the K230 script from CanMV IDE on the LCKFB K230 board.
- Re-check `git status` and the handoff snapshot before assuming the worktree or
  hardware state is unchanged.
