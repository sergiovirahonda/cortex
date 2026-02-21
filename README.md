# Cortex

A professional-grade ESP32-S3 flight controller for DIY quadcopter drones, featuring real-time attitude stabilization, DShot ESC control, and wireless command reception. This project pairs with [Synapse](https://github.com/sergiovirahonda/synapse), the transmitter controller that sends flight commands via joystick.

## Overview

Cortex is the flight control system (FC) for your DIY drone project. It receives wireless commands from the Synapse transmitter, processes sensor data from an MPU6050 gyroscope/accelerometer, and uses PID control algorithms to stabilize the drone while executing flight maneuvers. The controller runs on an ESP32-S3 microcontroller and communicates with a 4-in-1 ESC via native DShot protocol.

## Features

* 🚁 **Real-time Stabilization**: PD controllers for roll/pitch, P controller for yaw
* ⚡ **Native DShot ESC Control**: Direct hardware-level DShot600 protocol via ESP32 RMT
* 📡 **Wireless Communication**: nRF24L01 radio module for low-latency command reception
* 🎯 **MPU6050 Integration**: Hardware DLPF filtering and software calibration for accurate attitude sensing
* 📺 **OLED Telemetry Display**: Real-time flight data on the drone (attitude, throttle, motor outputs, trims)
* 📤 **Radio Telemetry Downlink**: Sends throttle, roll, and pitch back to the transmitter via nRF24 ACK payload (for Synapse display)
* 🔒 **Safety Features**: Arming sequence, throttle limits, and hardware initialization checks
* 🧵 **Dual-Core Design**: Flight loop on core 1 (~1 kHz); avionics task on core 0 samples LIDAR, GPS, compass with cross-core mutex for thread-safe reads
* 📏 **Optional Avionics**: TF-Luna LIDAR, GPS (UART), and compass (I2C) supported; adapters are null-safe so sensors can be omitted per build
* 🏗️ **Clean Architecture**: Modular design with adapters, models, controllers, and tasks
* ⚙️ **PlatformIO Integration**: Modern build system with dependency management

## Hardware Requirements

### Core Components

* **ESP32-S3-DevKitC-1** (16MB flash, 8MB PSRAM recommended)
* **4-in-1 ESC** (DShot600 compatible)
* **3S LiPo Battery** (11.1V nominal)
* **MPU6050** (6-axis gyroscope/accelerometer)
* **nRF24L01** radio module
* **SSD1306 OLED Display** (128x64, I2C)
* **4x Brushless Motors** (matching your ESC specifications)

### Pin Connections

| Component | Pin | ESP32-S3 Pin | Notes |
|-----------|-----|--------------|-------|
| **I2C Bus 0** (OLED & Compass) | | | |
| I2C SDA | - | GPIO 1 | OLED, compass |
| I2C SCL | - | GPIO 2 | OLED, compass |
| OLED RST | - | GPIO 21 | Optional reset |
| **I2C Bus 1** (MPU6050, flight-critical) | | | |
| MPU SDA | - | GPIO 17 | Dedicated for gyro polling |
| MPU SCL | - | GPIO 18 | 400 kHz |
| **UART** | | | |
| TF-Luna RX | - | GPIO 39 | UART1 (LIDAR) |
| TF-Luna TX | - | GPIO 40 | UART1, 115200 |
| GPS RX | - | GPIO 41 | UART2 |
| GPS TX | - | GPIO 42 | UART2, 115200 |
| **Radio (SPI)** | | | |
| nRF24L01 CE | - | GPIO 14 | Chip Enable |
| nRF24L01 CSN | - | GPIO 9 | Chip Select |
| nRF24L01 MOSI | - | GPIO 11 | SPI MOSI |
| nRF24L01 MISO | - | GPIO 12 | SPI MISO |
| nRF24L01 SCK | - | GPIO 13 | SPI Clock |
| **Motors (DShot)** | | | |
| Motor 1 (Rear Right, CCW) | - | GPIO 4 | RMT Channel 0 |
| Motor 2 (Front Right, CW) | - | GPIO 5 | RMT Channel 1 |
| Motor 3 (Rear Left, CW) | - | GPIO 6 | RMT Channel 2 |
| Motor 4 (Front Left, CCW) | - | GPIO 7 | RMT Channel 3 |

### Motor Configuration

The drone uses a standard X-frame quadcopter configuration:

```
    M4 (FL)    M2 (FR)
       \      /
        \    /
         \  /
          \/
         /\
        /  \
       /    \
      /      \
    M3 (RL)    M1 (RR)
```

* **M1**: Rear Right, Counter-Clockwise (CCW)
* **M2**: Front Right, Clockwise (CW)
* **M3**: Rear Left, Clockwise (CW)
* **M4**: Front Left, Counter-Clockwise (CCW)

## Software Architecture

```
cortex/
├── src/
│   ├── main.cpp                    # Main loop (core 1), setup, init
│   ├── config/
│   │   ├── drone_config.*          # PID gains, throttle bands, limits, trim, radio address
│   │   └── pins.h                  # GPIO and bus definitions
│   ├── controllers/
│   │   ├── flight_controller.*     # Throttle stages, mixing matrix, trims, failsafe
│   │   └── display_controller.*    # OLED output (gated by feature flag)
│   ├── tasks/
│   │   └── avionics_task.*         # FreeRTOS task (core 0): LIDAR/GPS/compass, mutex-protected
│   ├── adapters/
│   │   ├── radio_adapter.*         # nRF24L01: receive commands, send telemetry (ACK payload)
│   │   ├── mpu_adapter.*           # MPU6050 sensor interface
│   │   ├── motor_adapter.*         # DShot ESC control (native)
│   │   ├── display_adapter.*      # SSD1306 OLED
│   │   ├── tf_luna_adapter.*      # TF-Luna LIDAR (UART)
│   │   ├── gps_adapter.*          # GPS (UART)
│   │   ├── compass_adapter.*      # Compass (I2C bus 0)
│   │   └── bmp280_adapter.*        # BME280 barometer (optional, feature-flagged)
│   └── models/
│       ├── attitude.*              # Attitude state & PID (roll/pitch/yaw)
│       ├── drone_command.*        # DronePacket (commands), TelemetryData (downlink)
│       ├── motor_output.*          # Motor speed outputs
│       ├── avionics_params.h       # Adapter pointers for avionics task
│       ├── avionics_metrics.h       # LidarReadings, GpsReadings, CompassReadings
│       └── ...
└── platformio.ini                  # Build configuration
```

### Key Components

* **FlightController**: Throttle-band logic (launch / transition / flight), motor mixing matrix, trim updates, failsafe
* **Attitude**: Sensor state and PID: roll/pitch (PD + I in flight), yaw (P only); launch vs flight gains via blend
* **DroneConfig**: All tunable constants (PID gains, throttle bands, limits, trim steps, radio address) in one place
* **RadioAdapter**: nRF24L01 init, receive `DronePacket` (commands), send `TelemetryData` as ACK payload
* **MPUAdapter**: MPU6050 interface, calibration, filtering (on I2C bus 1 for minimal interference)
* **NativeDShotMotorAdapter**: ESP32 RMT-based DShot600 for ESC control
* **Avionics task**: FreeRTOS task pinned to **core 0**. Samples LIDAR, GPS, and compass at ~100 Hz; updates a global `AvionicsMetrics` struct under a **FreeRTOS mutex**. The main loop on **core 1** calls `getAvionicsMetrics(localAvionics)` for a thread-safe copy. Adapter pointers are null-checked so optional sensors can be omitted.

### Control Loop

**Core 1** runs the flight loop at **~1 kHz**:

0. **Thread-safe avionics snapshot**: `getAvionicsMetrics(localAvionics)` (mutex-protected copy of LIDAR/GPS/compass; reserved for future altitude/position/heading use).
1. **Receive Radio**: Check for new command packet from transmitter
2. **Update command & send telemetry**: If packet received, load command, remap, then send telemetry (throttle, roll, pitch) back via ACK payload
3. **Failsafe**: Update failsafe state; override command if link lost
4. **Read Sensors**: Update MPU6050 attitude (roll, pitch, yaw rates)
5. **Calculate PID**: Three throttle stages (launch = high Kp, transition = blend, flight = full PID); trim added to setpoint
6. **Mix Motors**: Apply mixing matrix (throttle + corrections)
7. **Write Motors**: DShot to all 4 ESCs
8. **Update Trims**: Process trim buttons from command (debounced)

The OLED is updated at **~1 Hz** only when throttle is below 300 (idle), to avoid I2C bus contention with the flight loop. Loop frequency, attitude, throttle, motor outputs, and trims are shown.

## Installation

### Prerequisites

1. **PlatformIO**: Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) or use the CLI
2. **USB Cable**: For uploading firmware to ESP32-S3
3. **Synapse Transmitter**: Ensure the [Synapse](https://github.com/sergiovirahonda/synapse) transmitter is configured with matching radio address

### Setup Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/sergiovirahonda/cortex.git
   cd cortex
   ```

2. **Install dependencies**
   
   PlatformIO will automatically install required libraries:
   * `RF24` - nRF24L01 radio driver
   * `MPU6050_light` - MPU6050 sensor library
   * `Adafruit SSD1306` - OLED display driver
   * `Adafruit GFX Library` - Graphics library for display

3. **Configure radio address**
   
   Edit `src/config/drone_config.cpp` so the RX address matches your Synapse transmitter:
   ```cpp
   memcpy(radioAddress, "00001", 6);  // Must match transmitter pipe address
   ```

4. **Build and upload**
   ```bash
   pio run -t upload
   ```

5. **Monitor serial output**
   ```bash
   pio device monitor
   ```
   
   You should see initialization messages, calibration progress, and arming sequence.

## Configuration

### Radio Settings

The radio adapter is configured to match the Synapse transmitter:

* **Data Rate**: 250KBPS (longest range)
* **Power Level**: PA_HIGH (match Synapse TX for range)
* **Channel**: 108 (above 2.48GHz to avoid WiFi interference)
* **Auto-ACK**: Enabled with ACK payload (used for telemetry downlink)

### Avionics Task (LIDAR, GPS, Compass)

Optional sensors (TF-Luna LIDAR, GPS module, compass) are sampled on **core 0** in a FreeRTOS task. Before the task starts, `initAvionicsMutex()` is called so the main loop can safely read a snapshot with `getAvionicsMetrics()`. The task only updates adapters that are non-null in `AvionicsParams`, so you can omit a sensor by not wiring it and leaving its pointer null in your build (the default setup assigns all three). Avionics data is currently collected for future use (e.g. altitude hold, position hold).

### Telemetry (Downlink to Transmitter)

Cortex sends a small telemetry payload back to the transmitter on every received command packet, using the nRF24 **ACK payload** (no extra round-trip).

**Wire format** (`TelemetryPacket` in `src/models/drone_command.h`):

| Field  | Type    | Description                    |
|--------|---------|--------------------------------|
| `pwm`  | int16_t | Current throttle (command)     |
| `roll` | int16_t | Current roll angle (degrees)   |
| `pitch`| int16_t | Current pitch angle (degrees)  |

**Flow:**
1. Transmitter (Synapse) sends a `DronePacket` (commands).
2. Cortex receives it, updates command, then calls `radioAdapter.sendTelemetry(telemetry)`.
3. The radio sends the telemetry as the ACK payload for pipe 1 (6-byte fixed payload).
4. Synapse reads the ACK payload and displays throttle, roll, and pitch on the TX screen.

**Implementation:** When a packet is received (`main.cpp`), the fast loop sets `telemetry.setPwm(command.getThrottle())`, `setRoll(attitude.getRollAngle())`, `setPitch(attitude.getPitchAngle())`, then `radioAdapter.sendTelemetry(telemetry)`. LIDAR, GPS, and compass are sampled in the avionics task but are not currently sent over telemetry (reserved for future use).

### PID Tuning

All gains and limits are in **`src/config/drone_config.cpp`** (and declared in `drone_config.h`).

* **Flight stage (stage 3)**: Full PID for roll/pitch (Kp, Ki, Kd), P-only for yaw. Used when throttle is above the flight threshold.
* **Launch stage (stage 1)**: Higher Kp/Kd for roll/pitch (PD only, no I) for spool-up; separate `rollLaunchKp`, `pitchLaunchKp`, etc.
* **Transition (stage 2)**: Linear blend of launch and flight gains between two throttle values; I-term can be enabled in the upper part of the band.

Throttle band constants: `throttleIdle`, `throttleLaunchEnd`, `throttleFlightStart`. Adjust these and the PID/launch gains to match your frame and props.

**Tuning guidelines:**
* **Kp**: Stiffness. Too low = sluggish, too high = oscillations.
* **Kd**: Damping. Reduces overshoot; keep Kd &lt; Kp to avoid noise amplification.
* **Ki**: Removes steady-state error; cap with `maxIOutput` to avoid windup.
* Tune stage 1 (launch) for stable level at low throttle; stage 3 (flight) for smooth hover and minimal wobble.

### MPU6050 Calibration

The MPU6050 uses hardware DLPF (Digital Low-Pass Filter) set to 44 Hz bandwidth to reduce vibration noise. Software filtering and accel offsets are configured in `src/adapters/mpu_adapter.cpp`. Accel offsets can also be set in `src/config/drone_config.cpp` (e.g. `accelOffsets.xOffset`, `yOffset`, `zOffset`).

**Calibration process:**
1. Place drone on level surface
2. Power on and wait for gyro calibration to complete
3. If angles drift, adjust accel offsets in config or MPU adapter
4. If angles jump when motors spin, increase gyro filter coefficient (e.g. 0.99)

### Motor Mixing

The motor mixing matrix in `src/controllers/flight_controller.cpp` combines throttle and PID corrections:

* **Pitch**: Nose down → Front motors (M2, M4) increase
* **Roll**: Left side down → Left motors (M3, M4) increase
* **Yaw**: Turn right (CW) → Clockwise motors (M2, M3) increase

Trim is applied as a **setpoint offset** (not in the mix): desired angle = command + trim. Use the transmitter trim buttons to adjust; step size and debounce are in `drone_config.cpp` (`trimStepPitchRollDeg`, `trimStepYawDegPerSec`, `trimDelay`).

### Safety Limits

Motor outputs are clamped to safe ranges:

```cpp
const int MIN_THROTTLE_PWM = 50;   // Minimum DShot value
const int MAX_THROTTLE_PWM = 2047; // Maximum DShot value
```

DShot protocol enforces a minimum value of 48 for safety (motors won't spin below this).

## Usage

### Arming Sequence

The flight controller performs an automatic arming sequence on startup:

1. **Initialization** (5 seconds)
   * I2C bus setup
   * OLED display initialization
   * Radio hardware check
   * MPU6050 calibration

2. **Motor Wake-up** (3 seconds)
   * Sends zero throttle to all ESCs
   * Prepares ESCs for DShot communication

3. **Friction Kick** (100ms)
   * Brief high throttle pulse to overcome motor friction
   * Ensures motors are ready to respond immediately

4. **Idle State**
   * Motors spin at low idle speed (200 DShot units)
   * Drone is ready to receive commands

**⚠️ WARNING**: Keep clear of propellers during arming sequence!

### Flight Operation

1. **Power on** the drone (3S LiPo battery)
2. **Wait** for arming sequence to complete (OLED shows "ARMING MOTORS... STAND CLEAR!" then flight metrics when idle)
3. **Verify** radio connection (check that commands are received)
4. **Control** the drone using the Synapse transmitter
5. **Monitor** telemetry on OLED display:
   * Pitch/Roll angles
   * Yaw rate
   * Throttle percentage
   * Individual motor speeds

### OLED Display

The display shows real-time flight data updated at **1 Hz** (once per second):

```
- @ 1000 Hz -
P: 2.5 | R: -1.2
Y: 0.8 | T: 1050
M1: 250 | M2: 280
M3: 240 | M4: 270
Trims: P: 0.2 | R: -0.4 | Y: 0
```

## Troubleshooting

### Radio Not Receiving Commands

* Verify nRF24L01 wiring (CE, CSN, SPI pins)
* Check that radio address matches Synapse transmitter
* Ensure transmitter is powered and sending commands
* Verify radio channel and data rate settings match

### MPU6050 Not Initializing

* Check I2C wiring (SDA, SCL)
* Verify MPU6050 is powered (3.3V)
* Ensure I2C address is correct (0x68)
* Check serial output for specific error messages

### Motors Not Spinning

* Verify DShot signal wiring (GPIO 4, 5, 6, 7)
* Check ESC power supply (3S LiPo connected)
* Ensure ESCs are DShot600 compatible
* Verify arming sequence completed successfully
* Check that throttle commands are being received

### Drone Oscillating or Unstable

* **Reduce PID gains**: Lower Kp and Kd values in `src/config/drone_config.cpp`
* **Increase DLPF**: Adjust hardware filter or increase software filter coefficient
* **Check motor balance**: Ensure all motors spin smoothly
* **Verify propellers**: Check for damage or incorrect mounting

### Drone Drifting

* **Calibrate MPU6050**: Re-run calibration on level surface
* **Adjust motor mixing**: Add trim values in `flight_controller.cpp`
* **Check motor direction**: Verify all motors spin in correct direction
* **Balance propellers**: Unbalanced props cause vibration and drift

## Development

### Building

```bash
# Build project
pio run

# Upload to device
pio run -t upload

# Clean build artifacts
pio run -t clean

# Monitor serial output
pio device monitor
```

### Code Structure

* **Adapters**: Hardware interface layers (radio, MPU, motors, display, LIDAR, GPS, compass)
* **Models**: Data structures and business logic (attitude, commands, outputs, avionics metrics)
* **Controllers**: High-level control algorithms (flight controller, display)
* **Tasks**: FreeRTOS tasks (e.g. avionics task on core 0 with mutex-protected shared state)

### Adding Features

* **Altitude Hold**: BME280 adapter exists (feature-flagged); add altitude PID loop
* **GPS / LIDAR / Compass on Telemetry**: Avionics task already samples these; add fields to `TelemetryPacket` and Synapse display (keep packet ≤ 16 bytes for fixed payload), or use onboard only (e.g. altitude hold, position hold)
* **Telemetry Downlink**: Implemented (throttle, roll, pitch via ACK payload; 6-byte fixed packet)
* **LED Indicators**: Add status LEDs for visual feedback

## Related Projects

* **[Synapse](https://github.com/sergiovirahonda/synapse)**: The transmitter controller that sends commands to Cortex

## Safety Disclaimer

⚠️ **WARNING**: This is experimental software for DIY drones. Always:

* Test in a safe, open area away from people and property
* Wear safety glasses when testing
* Start with low throttle and gradually increase
* Keep hands and body clear of propellers at all times
* Ensure proper battery handling and charging safety
* Follow local regulations regarding drone operation

The authors are not responsible for any damage or injury resulting from the use of this software.

## License

This project is open source. See repository for license details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**Built with ❤️ for the DIY drone community**
