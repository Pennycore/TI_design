#ifndef COMPETITION_CONFIG_H_
#define COMPETITION_CONFIG_H_

/*
 * Change only this value before building:
 *   2: fast one-lap tracking and stop at A (no ball)
 *   4: A-to-B carrying-ball run
 *   5: one-lap carrying-ball run
 *   6: one-lap carrying-ball run at the referee-specified beam position
 *
 * The current top-level state machine still implements the task-2 one-lap
 * finish. Modes 4/5/6 below already select their safe motion profile; their
 * different ball targets/end conditions are handled by the later ball task
 * state machine.
 */
#define COMPETITION_TASK_MODE                  (2U)

#if (COMPETITION_TASK_MODE == 2U)

#define COMPETITION_CRUISE_SPEED               (18.0f)
#define COMPETITION_CURVE_SLOWDOWN             (5.0f)
#define COMPETITION_SOFT_START_ENABLE           (0U)
#define COMPETITION_BASE_ACCEL_STEP             (100.0f)
#define COMPETITION_OUTPUT_RISE_STEP            (100.0f)
#define COMPETITION_OUTPUT_FALL_STEP            (100.0f)
#define COMPETITION_SEGMENTED_SPEED_FF_ENABLE    (0U)

#elif ((COMPETITION_TASK_MODE == 4U) || \
       (COMPETITION_TASK_MODE == 5U) || \
       (COMPETITION_TASK_MODE == 6U))

/*
 * At 17 encoder counts/10 ms the theoretical lap time is about 25.8 s.
 * This leaves usable margin below 30 s while avoiding abrupt acceleration
 * of the ball. The base target reaches 17 in roughly 1.0 s.
 */
#define COMPETITION_CRUISE_SPEED               (17.0f)
#define COMPETITION_CURVE_SLOWDOWN             (6.0f)
#define COMPETITION_SOFT_START_ENABLE           (1U)
#define COMPETITION_BASE_ACCEL_STEP             (0.15f)
#define COMPETITION_OUTPUT_RISE_STEP            (1.0f)
#define COMPETITION_OUTPUT_FALL_STEP            (3.0f)
#define COMPETITION_SEGMENTED_SPEED_FF_ENABLE    (1U)

#else
#error "COMPETITION_TASK_MODE must be 2, 4, 5, or 6"
#endif

#endif /* COMPETITION_CONFIG_H_ */
