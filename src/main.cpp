#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <RF24.h>
#include "driver/rmt.h" 

// Internal imports
#include "models/attitude.h"
#include "models/motor_output.h"
#include "models/drone_command.h"
#include "adapters/radio_adapter.h"
#include "adapters/mpu_adapter.h"
#include "adapters/motor_adapter.h"
#include "controllers/flight_controller.h"


// ---------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------
// Radio & screen
#define I2C_SDA 17
#define I2C_SCL 18
#define OLED_RST 21
#define RADIO_CE_PIN 14
#define RADIO_CSN_PIN 9

// Motors
#define M1_PIN 4  // Motor 1 (Rear Right): Spins CCW
#define M2_PIN 5  // Motor 2 (Front Right): Spins CW
#define M3_PIN 6  // Motor 3 (Rear Left): Spins CW
#define M4_PIN 7  // Motor 4 (Front Left): Spins CCW

// ---------------------------------------------------------
// SCREEN DEFINITIONS
// ---------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C // Almost all 0.96" OLEDs are 0x3C
#define MPU_ADDR 0x68

// =================================================================
// MPU-6050 OBJECT AND VARIABLES
// =================================================================

// Initialize the MPU-6050 sensor object
MPU6050 mpu(Wire);

// =================================================================
// RADIO CONFIGURATIONS
// =================================================================
byte NRF_RX_ADDRESS[6] = "00001";

// =================================================================
// ADAPTER AND CONTROLLER OBJECTS
// =================================================================

DronePacket packet;
Attitude attitude;
DroneCommand command(0, 0, 0, 0);
MotorOutput motorOutput;
RadioAdapter radioAdapter(
    RADIO_CE_PIN,
    RADIO_CSN_PIN,
    NRF_RX_ADDRESS
);
MPUAdapter mpuAdapter(&mpu);
FlightController flightController;
NativeDShotMotorAdapter esc1, esc2, esc3, esc4;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
  Serial.begin(115200);

  // 1. Initialize I2C with your specific pins
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed. Checking I2C connection..."));
    for(;;); // Don't proceed, loop forever
  }
  // 4. Set Text Properties
  display.clearDisplay();
  display.invertDisplay(false);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0,0); 

  // 2. Initialize Drivers (BUT DO NOT ARM YET)
  display.println("Initializing adapters...");
  display.display();

  // Initialize ESCs
  esc1.init(M1_PIN, RMT_CHANNEL_0);
  esc2.init(M2_PIN, RMT_CHANNEL_1);
  esc3.init(M3_PIN, RMT_CHANNEL_2);
  esc4.init(M4_PIN, RMT_CHANNEL_3);

  // Initialize radio & MPU drivers
  radioAdapter.begin();
  if (!radioAdapter.isChipConnected()){
    Serial.println("Radio Hardware Fail!");
    display.println("Radio: FAIL");
    display.println("Check Wiring!");
    display.display();
    while (1); // Stop here
  }
  mpuAdapter.begin();
  display.println("Radio status: OK");
  display.println("MPU status: OK");
  display.display();
  delay(5);

  display.println("Initializing DLPF...");
  display.display();
  // ENABLE HARDWARE DLPF (The "Vibration Shield")
  // We write directly to the MPU register 0x1A
  Wire.beginTransmission(0x68); // MPU Address
  Wire.write(0x1A);             // Register: CONFIG
  Wire.write(0x03);             // Value: DLPF_CFG = 3 (44Hz bandwidth)
  Wire.endTransmission();
  // TUNE SOFTWARE FILTER
  // 0.98 is default. 
  // Increase to 0.99 if you see angles jumping when motors spin.
  display.println("Calibrating MPU9260...");
  display.display();
  mpuAdapter.calibrate();
  display.println("Calibration completed.");
  display.display();

  // ============================================================
  // 5. ARMING SEQUENCE
  // ============================================================
  // The MPU is ready. The Radio is ready. 
  // Now we wake up the ESCs immediately before flying.
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("ARMING MOTORS...");
  display.println("STAND CLEAR!");
  display.display();

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

  display.println("Resetting PID...");
  attitude.resetPID();
}

void loop() {
  // ================================================================
  // 1. FAST LOOP (FLIGHT CRITICAL) - Runs every cycle (~1kHz)
  // ================================================================
  
  // A. Receive Radio (Check if new packet arrived)
  if (radioAdapter.receivePacket(packet)) {
    command.loadFromPacket(packet);
    command.remap();
  }

  // B. Read Sensors
  mpuAdapter.getAttitude(attitude);

  // // C. Calculate PID
  // float pitchPD = attitude.calculatePitchPD(command.getPitch());
  // float rollPD  = attitude.calculateRollPD(command.getRoll());
  // float yawPD   = attitude.calculateYawP(command.getYaw());

  float pitchPD, rollPD, yawPD;

  // 48 is our "Deadzone" (since we send 48 as min throttle in NativeDShot)
  if (command.getThrottle() < 60) { 
      // 1. If throttle is low, RESET the memory.
      attitude.resetPID();
      
      // 2. Force PID corrections to Zero.
      // This prevents the motors from twitching while on the ground.
      pitchPD = 0;
      rollPD  = 0;
      yawPD   = 0;
      
  } else {
      // 3. Only calculate PID if we are actually flying!
      pitchPD = attitude.calculatePitchPD(command.getPitch());
      rollPD  = attitude.calculateRollPD(command.getRoll());
      yawPD   = attitude.calculateYawP(command.getYaw());
  }
  

  // D. Mix Motors
  flightController.computeMotorOutput(motorOutput, command, rollPD, pitchPD, yawPD);

  // E. WRITE TO MOTORS
  esc1.sendThrottle(motorOutput.getMotor1Speed());
  esc2.sendThrottle(motorOutput.getMotor2Speed());
  esc3.sendThrottle(motorOutput.getMotor3Speed());
  esc4.sendThrottle(motorOutput.getMotor4Speed());

  // ================================================================
  // 2. SLOW LOOP (HUMAN INTERFACE) - Runs every 3ms (3Hz)
  // ================================================================
  static unsigned long lastScreenUpdate = 0;
  
  if (millis() - lastScreenUpdate > 300) { 
    
    // Refresh display
    display.clearDisplay();
    display.setCursor(0,0);
    // Print to Buffer
    display.println(F("-- SYSTEM RUNNING --"));
  
    // Print readings
    display.print("Pitch: "); display.print(attitude.getPitchAngle());
    display.print(" | Roll: "); display.println(attitude.getRollAngle());
    display.print("Yaw: "); display.print(attitude.getYawRate());
    display.print(" | Throttle: "); display.println(command.getThrottle());

    // Print motor output PWM values
    display.print("M1: "); display.print(motorOutput.getMotor1Speed());
    display.print(" | M2: "); display.println(motorOutput.getMotor1Speed());
    // Print motor output PWM values
    display.print("M3: "); display.print(motorOutput.getMotor3Speed());
    display.print(" | M4: "); display.println(motorOutput.getMotor3Speed());
    
    display.display();
    lastScreenUpdate = millis();
  }
}