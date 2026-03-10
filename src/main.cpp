#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <RF24.h>
#include "driver/rmt.h" 

// Internal imports
#include "config/drone_config.h"
#include "models/attitude.h"
#include "models/altitude.h"
#include "models/motor_output.h"
#include "models/drone_command.h"
#include "adapters/radio_adapter.h"
#include "adapters/nrf24_radio_adapter.h"
#include "adapters/mpu_adapter.h"
#include "adapters/mpu6050_adapter.h"
#include "adapters/motor_adapter.h"
#include "adapters/barometer_adapter.h"
#include "adapters/bmp280_adapter.h"
#include "adapters/display_adapter.h"
#include "adapters/ssd1306_display_adapter.h"
#include "adapters/lidar_adapter.h"
#include "adapters/tf_luna_adapter.h"
#include "adapters/gps_adapter.h"
#include "adapters/beitian_be880_gps_adapter.h"
#include "adapters/compass_adapter.h"
#include "adapters/qmc5883l_adapter.h"
#include "adapters/current_sensor_adapter.h"
#include "adapters/ina219_current_sensor_adapter.h"
#include "controllers/flight_controller.h"
#include "controllers/display_controller.h"
#include "controllers/blackbox_controller.h"
#include "adapters/storage_adapter.h"
#include "adapters/sd_card_storage_adapter.h"
#include "tasks/avionics_task.h"
#include "tasks/radio_task.h"
#include "tasks/blackbox_task.h"
#include "models/blackbox_frame.h"
#include "config/pins.h"
#include "utils/loop_utils.h"

// =================================================================
// MPU-6050 OBJECT AND VARIABLES
// =================================================================

// Initialize the MPU-6050 sensor object
MPU6050 mpu(Wire1);

// =================================================================
// ADAPTER AND CONTROLLER OBJECTS
// =================================================================

DroneConfig droneConfig;
DroneCommand command(0, 0, 0, 0, 0, 0, 0, 0, 0);
MotorOutput motorOutput;
Attitude attitude(droneConfig);
Altitude altitude(droneConfig);
AttitudeTrim attitudeTrim;
AltitudeHold altitudeHold;
Nrf24RadioAdapter radioImpl(
    RADIO_CE_PIN,
    RADIO_CSN_PIN,
    const_cast<byte*>(droneConfig.getRadioAddress())
);
RadioAdapter* radioAdapter = &radioImpl;

// IMU (MPU6050)
Mpu6050Adapter mpuImpl(&mpu);
MPUAdapter* mpuAdapter = &mpuImpl;

// Barometer & current sensor (I2C: BMP280, INA219)
BMP280BarometerAdapter barometerImpl(&Wire);
BarometerAdapter* barometerAdapter = &barometerImpl;
Ina219CurrentSensorAdapter currentSensorImpl(&Wire);
CurrentSensorAdapter* currentSensorAdapter = &currentSensorImpl;

// Flight controller and ESCs
FlightController flightController(droneConfig);
NativeDShotMotorAdapter esc1, esc2, esc3, esc4;

// Display (OLED, I2C)
Ssd1306DisplayAdapter displayImpl(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST, OLED_ADDR);
DisplayAdapter* displayAdapter = &displayImpl;
DisplayController displayController(*displayAdapter, droneConfig);

// Serial peripherals: Lidar (TF-Luna) and GPS (Beitian BE880)
HardwareSerial TFLunaSerial(1);
HardwareSerial GpsSerial(2);
TFLunaLidarAdapter lidarImpl(&TFLunaSerial);
LidarAdapter* lidarAdapter = &lidarImpl;
BeitianBe880GpsAdapter gpsImpl(&GpsSerial, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
GpsAdapter* gps = &gpsImpl;

// Compass (QMC5883L, I2C)
QMC5883LCompassAdapter compassImpl;
CompassAdapter* compass = &compassImpl;

// Blackbox (SD_MMC 1-bit)
SdCardStorageAdapter sdStorage(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN);
BlackboxController blackboxController(&sdStorage, droneConfig);

static unsigned long lastScreenUpdate = 0;
static unsigned long lastLoopTime = 0;
static float loopFrequencyHz = 0.0f;
void setup() {
  Serial.begin(115200);

  // 1. Avionics Bus (Core 0)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  // 2. Flight Control Bus (Core 1)
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(400000); // Fast mode for rapid gyro polling

  // 2. Initialize OLED (only when display feature enabled)
  if (droneConfig.getFeatureFlagEnableDisplay()) {
    if (!displayAdapter->begin()) {
      Serial.println(F("SSD1306 allocation failed. Checking I2C connection..."));
      for (;;); // Halt on display init failure
    }
    displayAdapter->clearDisplay();
    displayAdapter->invertDisplay(false);
    displayAdapter->setTextSize(1);
    displayAdapter->setTextColor(DISPLAY_COLOR_WHITE);
  }
  displayController.setCursor(0, 0);
  displayController.println("Initializing adapters...");
  
  displayController.display();

  // Initialize ESCs
  esc1.init(M1_PIN, RMT_CHANNEL_0);
  esc2.init(M2_PIN, RMT_CHANNEL_1);
  esc3.init(M3_PIN, RMT_CHANNEL_2);
  esc4.init(M4_PIN, RMT_CHANNEL_3);

  // Initialize radio & MPU drivers
  radioAdapter->begin();
  if (radioAdapter->isChipConnected()) {
    displayController.println("Radio status: OK");
  } else {
    displayController.println("Radio: -- (optional, not connected)");
  }
  mpuAdapter->begin();
  barometerAdapter->begin();
  displayController.println("Radio status: OK");
  displayController.println("MPU status: OK");
  displayController.println(barometerAdapter->isConnected() ? "BMP280: OK" : "BMP280: --");
  currentSensorAdapter->begin();
  displayController.println(currentSensorAdapter->isConnected() ? "INA219: OK" : "INA219: --");
  displayController.display();
  delay(5);

  // Initialize LIDAR, GPS, and Compass
  displayController.println("Initializing LIDAR, GPS, and Compass...");
  displayController.display();
  compass->begin();
  TFLunaSerial.begin(115200, SERIAL_8N1, TF_LUNA_RX_PIN, TF_LUNA_TX_PIN);
  lidarAdapter->begin();
  gps->begin();
  delay(150);  // Allow sensors to settle before use.

  displayController.println("LIDAR, GPS, and Compass initialized.");
  displayController.display();

  displayController.println("Initializing DLPF...");
  displayController.display();
  // ENABLE HARDWARE DLPF (The "Vibration Shield")
  // We write directly to the MPU register 0x1A
  Wire1.beginTransmission(0x68); // MPU Address
  Wire1.write(0x1A);             // Register: CONFIG
  Wire1.write(0x03);             // Value: DLPF_CFG = 3 (44Hz bandwidth)
  Wire1.endTransmission();
  // TUNE SOFTWARE FILTER
  // 0.98 is default. 
  // Increase to 0.99 if you see angles jumping when motors spin.
  displayController.println("Calibrating MPU6050...");
  displayController.display();
  mpuAdapter->calibrate(droneConfig);
  displayController.println("Calibration completed.");
  displayController.display();

  // ============================================================
  // 5. ARMING SEQUENCE
  // ============================================================
  // The MPU is ready. The Radio is ready. 
  // Now we wake up the ESCs immediately before flying.

  displayController.clearDisplay();
  displayController.setCursor(0, 0);
  displayController.println("ARMING MOTORS...");
  displayController.println("STAND CLEAR!");
  displayController.display();

  // A. Wake up (Send 0) - 3 Seconds
  for(int i=0; i<300; i++) {
    esc1.sendThrottle(0);
    esc2.sendThrottle(0);
    esc3.sendThrottle(0);
    esc4.sendThrottle(0);
    delay(10);
  }

  // B. The Kick (Punch friction)
  for(int i=0; i<20; i++) { // 100ms punch
    esc1.sendThrottle(700);
    esc2.sendThrottle(700);
    esc3.sendThrottle(700);
    esc4.sendThrottle(700);
    delay(5);
  }

  // C. Settle to Idle
  esc1.sendThrottle(200); 
  esc2.sendThrottle(200); 
  esc3.sendThrottle(200); 
  esc4.sendThrottle(200);
  displayController.println("Resetting PID...");
  attitude.resetPID();

  // D. Init cross-core sync and launch core 0 tasks
  initAvionicsMutex();
  initRadioMutexes();
  static AvionicsParams avionicsParams;
  avionicsParams.lidar = lidarAdapter;
  avionicsParams.gps = gps;
  avionicsParams.compass = compass;
  avionicsParams.barometer = barometerAdapter;
  avionicsParams.currentSensor = currentSensorAdapter;
  startAvionicsTask(&avionicsParams, 0);
  startRadioTask(radioAdapter, 0);

  blackboxController.init();
  startBlackboxTask(&blackboxController, 0);
}

void loop() {

  // ================================================================
  // 0. THREAD-SAFE DATA FETCH (cross-core mutex)
  // ================================================================
  AvionicsMetrics localAvionics;
  getAvionicsMetrics(localAvionics);
  PowerSnapshot powerSnapshot;
  powerSnapshot.fromReadings(localAvionics.currentSensor);

  // ================================================================
  // 1. FAST LOOP (FLIGHT CRITICAL) - 1000Hz
  // ================================================================
  unsigned long now = micros();
  float loopDtSec = LoopUtils::computeLoopDtAndHz(now, lastLoopTime, loopFrequencyHz);

  // A. Get latest command from radio task (core 0)
  DronePacket packet;
  unsigned long lastPacketTime;
  bool packetReceived = getLatestRadioCommand(packet, lastPacketTime);
  if (packetReceived) {
    command.loadFromPacket(packet);
    command.remap();
    submitTelemetry(
      command.getThrottle(),
      (int16_t)(attitude.getRollAngle() * TELEMETRY_ANGLE_SCALE),
      (int16_t)(attitude.getPitchAngle() * TELEMETRY_ANGLE_SCALE),
      altitudeHold.getAltitudeHoldEngaged()
    );
  }
  
  // Update failsafe state (pipeline runs after sensors)
  flightController.updateFailsafe(packetReceived, command.getThrottle());

  // B. Read Sensors
  mpuAdapter->getAttitude(attitude);
  mpuAdapter->getRawAccelGs(altitude);
  altitude.updateSensors(
    attitude.getPitchAngle(),
    attitude.getRollAngle(),
    localAvionics.lidar.getDistanceCm()
  );
  attitude.setOnGround(command.getThrottle() < droneConfig.getThrottleIdle());
  attitude.updateHeading(
    (float)localAvionics.compass.getSmoothedAzimuth(),
    loopDtSec,
    localAvionics.compass.getCompassHealth()
  );

  // C. Altitude hold (then failsafe) may override command
  flightController.applyAltitudeHold(command, altitude, altitudeHold);

  // D. Calculate PID

  float pitchPD, rollPD, yawPD;

  flightController.computeAttitudeCorrections(command, attitude, attitudeTrim, pitchPD, rollPD, yawPD);

  // E. Mix Motors
  flightController.computeMotorOutput(motorOutput, command, attitudeTrim, rollPD, pitchPD, yawPD);

  // F. WRITE TO MOTORS
  esc1.sendThrottle(motorOutput.getMotor1Speed());
  esc2.sendThrottle(motorOutput.getMotor2Speed());
  esc3.sendThrottle(motorOutput.getMotor3Speed());
  esc4.sendThrottle(motorOutput.getMotor4Speed());

  // Update trims from command (debounced inside FlightController)
  flightController.updateTrims(command, attitudeTrim);

  // Blackbox: enqueue frame (non-blocking; drops when queue full)
  BlackboxFrame frame;
  frame.populate(attitude, command, localAvionics, motorOutput, powerSnapshot, millis());
  (void)blackboxController.tryEnqueue(frame);

  // ================================================================
  // 2. DISPLAY LOOP (HUMAN INTERFACE) - Ground Only!
  // ================================================================

  // Only update screen if throttle is at absolute zero to prevent PID stutter
  if (command.getThrottle() < 300) {
    if (millis() - lastScreenUpdate > 1000) {
      // It's safe to pause the PID loop because the motors are idle
      displayController.displayFlightMetrics(
        loopFrequencyHz, attitude, command, motorOutput, attitudeTrim
      );
      lastScreenUpdate = millis();
    }
  } else {
    // Optional: clear screen or show a static "ARMED" message once, 
    // then stop touching the I2C bus completely.
  }
}