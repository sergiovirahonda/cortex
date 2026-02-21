#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <RF24.h>
#include "driver/rmt.h" 

// Internal imports
#include "config/drone_config.h"
#include "models/attitude.h"
#include "models/motor_output.h"
#include "models/drone_command.h"
#include "adapters/radio_adapter.h"
#include "adapters/mpu_adapter.h"
#include "adapters/motor_adapter.h"
#include "adapters/bmp280_adapter.h"
#include "adapters/display_adapter.h"
#include "adapters/tf_luna_adapter.h"
#include "adapters/gps_adapter.h"
#include "adapters/compass_adapter.h"
#include "controllers/flight_controller.h"
#include "controllers/display_controller.h"
#include "tasks/avionics_task.h"
#include "config/pins.h"

// =================================================================
// MPU-6050 OBJECT AND VARIABLES
// =================================================================

// Initialize the MPU-6050 sensor object
MPU6050 mpu(Wire1);

// =================================================================
// ADAPTER AND CONTROLLER OBJECTS
// =================================================================

DroneConfig droneConfig;
DronePacket packet;
DroneCommand command(0, 0, 0, 0, 0, 0, 0, 0);
TelemetryData telemetry(0, 0, 0);
MotorOutput motorOutput;
Attitude attitude(droneConfig);
AttitudeTrim attitudeTrim;
RadioAdapter radioAdapter(
    RADIO_CE_PIN,
    RADIO_CSN_PIN,
    const_cast<byte*>(droneConfig.getRadioAddress())
);
MPUAdapter mpuAdapter(&mpu);
BME280Adapter bme280Adapter(&Wire);
FlightController flightController;
NativeDShotMotorAdapter esc1, esc2, esc3, esc4;
DisplayAdapter displayAdapter(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST, OLED_ADDR);
HardwareSerial TFLunaSerial(1);
HardwareSerial GpsSerial(2);
TFLunaAdapter tfLuna(&TFLunaSerial);
GpsAdapter gps(&GpsSerial, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
CompassAdapter compass;
DisplayController displayController(displayAdapter, droneConfig);

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
    if (!displayAdapter.begin()) {
      Serial.println(F("SSD1306 allocation failed. Checking I2C connection..."));
      for (;;); // Don't proceed, loop forever
    }
    displayAdapter.clearDisplay();
    displayAdapter.invertDisplay(false);
    displayAdapter.setTextSize(1);
    displayAdapter.setTextColor(DISPLAY_COLOR_WHITE);
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
  radioAdapter.begin();
  if (!radioAdapter.isChipConnected()) {
    displayController.println("Radio: FAIL");
    displayController.println("Check Wiring!");
    displayController.display();
    while (1); // Stop here
  }
  mpuAdapter.begin();
  bme280Adapter.begin();
  displayController.println("Radio status: OK");
  displayController.println("MPU status: OK");
  displayController.println(bme280Adapter.isConnected() ? "BME280: OK" : "BME280: --");
  displayController.display();
  delay(5);

  // Initialize LIDAR, GPS, and Compass
  displayController.println("Initializing LIDAR, GPS, and Compass...");
  displayController.display();
  compass.begin();
  TFLunaSerial.begin(115200, SERIAL_8N1, TF_LUNA_RX_PIN, TF_LUNA_TX_PIN);
  tfLuna.begin();
  gps.begin();

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
  mpuAdapter.calibrate(droneConfig);
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

  // D. Init cross-core sync then launch the avionics task
  initAvionicsMutex();
  static AvionicsParams avionicsParams;
  avionicsParams.lidar = &tfLuna;
  avionicsParams.gps = &gps;
  avionicsParams.compass = &compass;
  startAvionicsTask(&avionicsParams, 0);
}

void loop() {

  // ================================================================
  // 0. THREAD-SAFE DATA FETCH (cross-core mutex)
  // ================================================================
  AvionicsMetrics localAvionics;
  getAvionicsMetrics(localAvionics);

  // ================================================================
  // 1. FAST LOOP (FLIGHT CRITICAL) - 1000Hz
  // ================================================================
  
  // Measure loop frequency (running average so display shows ~average Hz, not fast/slow alternation)
  unsigned long now = micros();
  if (lastLoopTime > 0) {
    unsigned long periodUs = now - lastLoopTime;
    if (periodUs > 0) {
      float instantHz = 1000000.0f / (float)periodUs;
      loopFrequencyHz = 0.995f * loopFrequencyHz + 0.005f * instantHz;  // smooth toward ~1200 Hz
    }
  }
  lastLoopTime = now;
  
  // A. Receive Radio (Check if new packet arrived)
  bool packetReceived = radioAdapter.receivePacket(packet);
  if (packetReceived) {
    command.loadFromPacket(packet);
    command.remap();
    telemetry.setPwm(command.getThrottle());
    telemetry.setRoll(attitude.getRollAngle());
    telemetry.setPitch(attitude.getPitchAngle());
    radioAdapter.sendTelemetry(telemetry);
  }
  
  // Update failsafe state and apply failsafe if needed
  // Pass current throttle to check if drone is flying before activating failsafe
  flightController.updateFailsafe(packetReceived, command.getThrottle());
  flightController.applyFailsafe(command);

  // B. Read Sensors
  mpuAdapter.getAttitude(attitude);

  // C. Calculate PID

  float pitchPD, rollPD, yawPD;

  flightController.computeAttitudeCorrections(droneConfig, command, attitude, attitudeTrim, pitchPD, rollPD, yawPD);

  // D. Mix Motors
  flightController.computeMotorOutput(motorOutput, command, attitudeTrim, rollPD, pitchPD, yawPD);

  // E. WRITE TO MOTORS
  esc1.sendThrottle(motorOutput.getMotor1Speed());
  esc2.sendThrottle(motorOutput.getMotor2Speed());
  esc3.sendThrottle(motorOutput.getMotor3Speed());
  esc4.sendThrottle(motorOutput.getMotor4Speed());

  // Update trims from command (debounced inside FlightController)
  flightController.updateTrims(command, attitudeTrim, droneConfig);

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