#ifndef PINS_H
#define PINS_H

// ---------------------------------------------------------
// NRF24 RADIO
// ---------------------------------------------------------
#define RADIO_CE_PIN  14
#define RADIO_CSN_PIN 9

// ---------------------------------------------------------
// MOTORS (DShot - safe on ESP32-S3)
// ---------------------------------------------------------
#define M1_PIN 4  // Motor 1 (Rear Right): Spins CCW
#define M2_PIN 5  // Motor 2 (Front Right): Spins CW
#define M3_PIN 6  // Motor 3 (Rear Left): Spins CW
#define M4_PIN 7  // Motor 4 (Front Left): Spins CCW

// ---------------------------------------------------------
// GPS (UART2)
// ---------------------------------------------------------
#define GPS_RX_PIN  41
#define GPS_TX_PIN  42
#define GPS_BAUD    115200

// ---------------------------------------------------------
// TF-LUNA LIDAR (UART1)
// ---------------------------------------------------------
#define TF_LUNA_RX_PIN  39
#define TF_LUNA_TX_PIN  40

// ---------------------------------------------------------
// I2C BUS 0 (OLED & COMPASS)
// ---------------------------------------------------------
#define I2C_SDA  1
#define I2C_SCL  2
#define OLED_RST 21

// ---------------------------------------------------------
// I2C BUS 1: MPU6050 (Flight Critical - Core 1)
// ---------------------------------------------------------
#define MPU_SDA  17
#define MPU_SCL  18

// ---------------------------------------------------------
// SCREEN DEFINITIONS
// ---------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C  // Almost all 0.96" OLEDs are 0x3C
#define MPU_ADDR 0x68

#endif
