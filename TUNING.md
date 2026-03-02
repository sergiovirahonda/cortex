# Tuning Guide

All tuning parameters are set via **build flags** in `platformio.ini` (copy from `platformio.ini.example` if needed). Change a value, rebuild, and flash. No code edits required.

Defaults live in `src/config/drone_config_defaults.h`; any flag you set in `platformio.ini` overrides the default.

---

## 1. Pins & hardware

| Flag | Description | When to change |
|------|-------------|----------------|
| `RADIO_CE_PIN`, `RADIO_CSN_PIN` | nRF24L01 CE and CSN GPIOs | Your wiring |
| `M1_PIN` … `M4_PIN` | DShot ESC signal pins (motors 1–4) | Your wiring |
| `GPS_RX_PIN`, `GPS_TX_PIN`, `GPS_BAUD` | GPS UART pins and baud rate | Your GPS module |
| `TF_LUNA_RX_PIN`, `TF_LUNA_TX_PIN` | TF-Luna LiDAR UART pins | Your wiring |
| `I2C_SDA`, `I2C_SCL` | Main I2C bus (compass, baro, OLED) | Your wiring |
| `MPU_SDA`, `MPU_SCL` | MPU6050 I2C bus (often separate for less noise) | Your wiring |
| `OLED_RST`, `OLED_ADDR` | Display reset pin and I2C address | Your display / wiring |
| `MPU_ADDR` | MPU6050 I2C address (usually 0x68) | Only if using a different address |
| `SCREEN_WIDTH`, `SCREEN_HEIGHT` | Display resolution (e.g. 128×64) | Your display |

---

## 2. Accelerometer offsets

| Flag | Description | When to change |
|------|-------------|----------------|
| `ACCEL_X_OFFSET`, `ACCEL_Y_OFFSET`, `ACCEL_Z_OFFSET` | Calibrated accel offsets (G). Used so “level” is consistent across power cycles. | After running an accel calibration; paste the saved offsets here. |

---

## 2b. MPU6050 gyro filter

The MPU6050 uses a complementary filter to fuse gyro and accel for roll/pitch angle. This coefficient sets the gyro weight (1 − coefficient is accel weight).

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `MPU_FILTER_GYRO_COEF` | Gyro coefficient in the complementary filter. Higher = trust gyro more (faster response, more noise). Lower = trust accel more (smoother, slower; accel drift shows up more). | 0.98–0.998 | Angle lags or feels sluggish → increase (e.g. 0.998). Noisy or jittery attitude → decrease (e.g. 0.98). Default 0.995 is a good middle ground. |

---

## 3. Attitude PID (roll, pitch, yaw) — flight phase

Used when throttle is **above** the transition zone (in steady flight). Lower gains = smoother; higher = stiffer, more responsive.

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `ROLL_KP`, `PITCH_KP` | Proportional: angle error → correction. | 5–15 (pitch often 20–30% higher than roll) | Oscillation → reduce. Sluggish → increase. |
| `ROLL_KI`, `PITCH_KI` | Integral: removes steady-state error and trim drift. | 0.8–2.0 | Slow drift → increase slightly. Wobble or wind-up → decrease. |
| `ROLL_KD`, `PITCH_KD` | Derivative: damps response, reduces overshoot. | 1.5–6.0 | Overshoot or bounce → increase. Noisy/jittery → decrease. |
| `YAW_KP`, `YAW_KI` | Yaw is a **rate** loop (PI only; no D). Stick = desired deg/s. | Kp 0.15–0.6, Ki 0.02–0.2 | Slow to reach desired rate → increase. Yaw wobble → decrease. |
| `YAW_KD` | Unused (yaw is PI-only). | 0 | Leave 0. |
| `YAW_FF` | Feedforward: adds `YAW_FF × desired_yaw_rate` to yaw output in flight. | 0.8–1.2 | Stick feels dead → increase. Yaw overshoots → decrease. |

---

## 4. Launch gains (high throttle / takeoff)

Stronger gains used when throttle is in the **launch band** (just above idle through early climb) for quick stabilization. Blended with flight gains in the transition zone.

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `ROLL_LAUNCH_KP`, `PITCH_LAUNCH_KP` | P gain in launch band. | 14–20 | Launch wobble → reduce. Slow to level → increase. |
| `ROLL_LAUNCH_KD`, `PITCH_LAUNCH_KD` | D gain in launch band. | 2.5–5.0 | Damp launch bounce. |
| `YAW_LAUNCH_KP` | Yaw P in launch band. | 2.5–4.0 | Same idea: snappy at low speed. |

---

## 5. Throttle bands

Three zones: **idle**, **launch** (high P, no I), **transition** (blend), **flight** (full PID). All in PWM units (e.g. 1000–2000).

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `THROTTLE_IDLE` | Below this = motors off, PIDs reset. | 50–100 | Match your ESC idle. |
| `THROTTLE_LAUNCH_END` | Top of “launch” zone; below this we use launch gains. | 1300–1400 | Where the craft actually lifts off. |
| `THROTTLE_FLIGHT_START` | Above this = full “flight” gains and I-term. Also min throttle when altitude hold is active. | 1400–1550 | Set just above your hover throttle so I-term and flight gains engage when airborne. |

---

## 6. Output limits

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `MAX_PD_OUTPUT` | Max absolute P and D contribution to motor mix (clamp). | 250–400 | Motors saturate in hard maneuvers → increase (within ESC range). |
| `MAX_I_OUTPUT` | Max absolute I-term (anti-windup). | 40–80 | Steady trim drift → allow more I. Oscillation or “locked” feel → reduce. |

---

## 7. Trim (in-flight trim buttons)

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `TRIM_DELAY` | Ms between trim steps (debounce). | 150–350 | Trim too sensitive → increase. |
| `TRIM_STEP_PITCH_ROLL_DEG` | Degrees added per trim step (pitch/roll). | 0.1–0.3 | Finer trim → decrease. |
| `TRIM_STEP_YAW_DEG_PER_SEC` | Yaw trim rate (deg/s) per step. | 1.0–3.0 | Same idea. |

---

## 8. Feature flags

| Flag | Description | Values |
|------|-------------|--------|
| `FEATURE_FLAG_ALTITUDE` | Enable altitude hold (LiDAR + baro/accel fusion). | 0 = off, 1 = on |
| `FEATURE_FLAG_DISPLAY` | Enable OLED display and status screens. | 0 = off, 1 = on |
| `FEATURE_FLAG_COMPASS_HEADING` | Use compass in heading. When **on** (1): gyro + compass fusion (init, on-ground, and in-flight correction). When **off** (0): pure MPU (gyro only; no compass). | 0 = gyro only, 1 = fusion (default) |

---

## 9. Yaw stick and yaw hold

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `MAX_YAW_RATE_DPS` | Max yaw rate (deg/s) at full stick. | 100–180 | Snappier turn → increase. Gentler → decrease. |
| `YAW_HOLD_KP` | When stick is centered: heading error (deg) × this = desired yaw rate (deg/s). | 1.0–3.0 | Hold overshoots or oscillates → reduce. Sluggish to correct heading → increase. |
| `YAW_DEADZONE_RATE_DPS` | Stick considered “centered” when \|desired rate\| < this (deg/s). | 3–10 | Jitter between hold and rate → increase. Hard to engage hold → decrease. |
| `YAW_FUSION_WEIGHT` | Base weight for compass correction of gyro-integrated heading. Actual weight is this × cos(roll)×cos(pitch) so correction is reduced when tilted. | 0.04–0.15 | Heading drifts away from compass → increase. Compass noise or mag interference pulls heading around → decrease. Default 0.08. |

---

## 10. Altitude hold (when `FEATURE_FLAG_ALTITUDE=1`)

### PID (altitude error → throttle correction)

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `ALT_KP` | P: altitude error → throttle correction. | 1.0–2.5 | Slow to hold altitude → increase. Bounce or oscillation → decrease. |
| `ALT_KI` | I: removes steady-state altitude error. | 0.02–0.08 | Drift up/down over time → increase. Climb/fall oscillation → decrease. |
| `ALT_KD` | D: damps altitude response. | 0.5–2.5 | Overshoot when changing altitude → increase. |
| `ALT_MAX_CORRECTION` | Max ± throttle correction from altitude PID (PWM). | 150–300 | Limit how much alt-hold can add/subtract from hover. |
| `ALT_MAX_I_OUTPUT` | Anti-windup for altitude I-term. | 30–80 | Same idea as attitude I. |

### Fusion (LiDAR + accel)

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `ALT_FUSION_KP`, `ALT_FUSION_KI` | Complementary filter: pull fused altitude toward LiDAR. | Kp 1.5–3.0, Ki 0.05–0.2 | Estimate lags or is noisy → adjust; usually leave near default first. |

### Hover and engagement

| Flag | Description | Typical range | Tweak if |
|------|-------------|---------------|----------|
| `ALT_HOVER_THROTTLE` | Throttle (PWM) at which the craft hovers; PID adds correction on top. | 1400–1550 | Set to your measured hover PWM. |
| `ALT_LIDAR_MIN_CM`, `ALT_LIDAR_MAX_CM` | LiDAR range (cm) used for fusion; outside this we ignore LiDAR. | Match your sensor (e.g. 25–800). | Reject bad readings. |
| `ALT_LIDAR_MIN_ENGAGE_CM`, `ALT_LIDAR_MAX_ENGAGE_CM` | Altitude (cm) bounds to **allow engaging** altitude hold. | e.g. 100–700 (1–7 m) | Prevent engage too low or too high. |

---

## Suggested tuning order

1. **Pins and hardware** — Match your wiring and hardware.
2. **Accel offsets** — Calibrate once and paste; keeps “level” consistent.
3. **MPU6050 gyro filter** — If roll/pitch angle feels laggy or noisy, adjust `MPU_FILTER_GYRO_COEF` (see §2b).
4. **Throttle bands** — Set `THROTTLE_LAUNCH_END` and `THROTTLE_FLIGHT_START` around your real lift-off and hover.
5. **Attitude PID (flight)** — Get stable hover: tune roll/pitch P first, then D, then I. Then yaw Kp/Ki and YAW_FF.
6. **Launch gains** — If takeoff is wobbly or slow to level, adjust launch Kp/Kd.
7. **Yaw hold and fusion** — If you use stick-centered heading lock: set `YAW_HOLD_KP` and `YAW_DEADZONE_RATE_DPS`. If heading drifts from compass or is noisy, tune `YAW_FUSION_WEIGHT` (see §9).
8. **Altitude hold** (if enabled) — Set `ALT_HOVER_THROTTLE` to measured hover; tune ALT_KP, then ALT_KI/KD; adjust engage and LiDAR ranges as needed.

Change one thing at a time, test, then move on. If the craft is unstable, reduce gains (especially P and I) before increasing.
