#ifndef DRONE_CONFIG_DEFAULTS_H
#define DRONE_CONFIG_DEFAULTS_H

// Defaults when not provided via platformio.ini build_flags.
// Override any of these in platformio.ini to customize without editing code.

#ifndef ACCEL_X_OFFSET
#define ACCEL_X_OFFSET 0.0350f
#endif
#ifndef ACCEL_Y_OFFSET
#define ACCEL_Y_OFFSET -0.0035f
#endif
#ifndef ACCEL_Z_OFFSET
#define ACCEL_Z_OFFSET -0.0600f
#endif
#ifndef ROLL_KP
#define ROLL_KP 6.0f
#endif
#ifndef ROLL_KI
#define ROLL_KI 1.5f
#endif
#ifndef ROLL_KD
#define ROLL_KD 2.0f
#endif
#ifndef PITCH_KP
#define PITCH_KP 9.0f
#endif
#ifndef PITCH_KI
#define PITCH_KI 1.0f
#endif
#ifndef PITCH_KD
#define PITCH_KD 5.0f
#endif
#ifndef YAW_KP
#define YAW_KP 0.15f
#endif
#ifndef YAW_KI
#define YAW_KI 0.03f
#endif
#ifndef YAW_KD
#define YAW_KD 0.0f   /* unused: yaw is PI-only; D on rate amplifies gyro noise */
#endif
#ifndef ROLL_LAUNCH_KP
#define ROLL_LAUNCH_KP 16.0f
#endif
#ifndef ROLL_LAUNCH_KD
#define ROLL_LAUNCH_KD 3.3f
#endif
#ifndef PITCH_LAUNCH_KP
#define PITCH_LAUNCH_KP 18.0f
#endif
#ifndef PITCH_LAUNCH_KD
#define PITCH_LAUNCH_KD 4.3f
#endif
#ifndef YAW_LAUNCH_KP
#define YAW_LAUNCH_KP 3.45f
#endif
#ifndef THROTTLE_IDLE
#define THROTTLE_IDLE 60
#endif
#ifndef THROTTLE_LAUNCH_END
#define THROTTLE_LAUNCH_END 1350
#endif
#ifndef THROTTLE_FLIGHT_START
#define THROTTLE_FLIGHT_START 1500
#endif
#ifndef MAX_PD_OUTPUT
#define MAX_PD_OUTPUT 300.0f
#endif
#ifndef MAX_I_OUTPUT
#define MAX_I_OUTPUT 60.0f
#endif
#ifndef TRIM_DELAY
#define TRIM_DELAY 250
#endif
#ifndef TRIM_STEP_PITCH_ROLL_DEG
#define TRIM_STEP_PITCH_ROLL_DEG 0.2f
#endif
#ifndef TRIM_STEP_YAW_DEG_PER_SEC
#define TRIM_STEP_YAW_DEG_PER_SEC 2.0f
#endif
#ifndef FEATURE_FLAG_ALTITUDE
#define FEATURE_FLAG_ALTITUDE 0
#endif
#ifndef FEATURE_FLAG_DISPLAY
#define FEATURE_FLAG_DISPLAY 0
#endif

#endif
