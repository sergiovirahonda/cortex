#ifndef PINS_DEFAULTS_H
#define PINS_DEFAULTS_H

// Defaults when not provided via platformio.ini build_flags.
// Override any of these in platformio.ini to customize without editing code.

// ---------------------------------------------------------
// NRF24 RADIO
// ---------------------------------------------------------
#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN  14
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN 9
#endif

// ---------------------------------------------------------
// MOTORS (DShot - safe on ESP32-S3)
// ---------------------------------------------------------
#ifndef M1_PIN
#define M1_PIN 4  // Motor 1 (Rear Right): Spins CCW
#endif
#ifndef M2_PIN
#define M2_PIN 5  // Motor 2 (Front Right): Spins CW
#endif
#ifndef M3_PIN
#define M3_PIN 6  // Motor 3 (Rear Left): Spins CW
#endif
#ifndef M4_PIN
#define M4_PIN 7  // Motor 4 (Front Left): Spins CCW
#endif

// ---------------------------------------------------------
// GPS (UART2)
// ---------------------------------------------------------
#ifndef GPS_RX_PIN
#define GPS_RX_PIN  41
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN  42
#endif
#ifndef GPS_BAUD
#define GPS_BAUD    115200
#endif

// ---------------------------------------------------------
// TF-LUNA LIDAR (UART1)
// ---------------------------------------------------------
#ifndef TF_LUNA_RX_PIN
#define TF_LUNA_RX_PIN  39
#endif
#ifndef TF_LUNA_TX_PIN
#define TF_LUNA_TX_PIN  40
#endif

// ---------------------------------------------------------
// I2C BUS 0 (OLED & COMPASS)
// ---------------------------------------------------------
#ifndef I2C_SDA
#define I2C_SDA  1
#endif
#ifndef I2C_SCL
#define I2C_SCL  2
#endif
#ifndef OLED_RST
#define OLED_RST 21
#endif

// ---------------------------------------------------------
// I2C BUS 1: MPU6050 (Flight Critical - Core 1)
// ---------------------------------------------------------
#ifndef MPU_SDA
#define MPU_SDA  17
#endif
#ifndef MPU_SCL
#define MPU_SCL  18
#endif

// ---------------------------------------------------------
// SCREEN DEFINITIONS
// ---------------------------------------------------------
#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 128
#endif
#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 64
#endif
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C  // Almost all 0.96" OLEDs are 0x3C
#endif
#ifndef MPU_ADDR
#define MPU_ADDR 0x68
#endif

#endif
