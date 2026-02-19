#ifndef DISPLAY_ADAPTER_H
#define DISPLAY_ADAPTER_H

#include <Arduino.h>
#include <Wire.h>

class Adafruit_SSD1306;

/**
 * Adapter for SSD1306 OLED display (e.g. 128x64 I2C).
 * Wraps Adafruit_SSD1306 and follows the same pattern as RadioAdapter, MPUAdapter.
 * Inherits Print so print()/println() work on the adapter.
 */
class DisplayAdapter : public Print {
public:
    DisplayAdapter(
        int width,
        int height,
        TwoWire* wire,
        int rstPin,
        uint8_t i2cAddr = 0x3C
    );
    ~DisplayAdapter();

    /** Initialize display; returns false on failure. */
    bool begin();

    void clearDisplay();
    void setCursor(int16_t x, int16_t y);
    void display();  // push buffer to screen
    void setTextSize(uint8_t s);
    void setTextColor(uint16_t color);
    void invertDisplay(bool i);

    /** Print interface: forward to underlying display. */
    size_t write(uint8_t c) override;

private:
    Adafruit_SSD1306* display_;
    int width_;
    int height_;
    TwoWire* wire_;
    int rstPin_;
    uint8_t i2cAddr_;
};

/** Common color for setTextColor (matches SSD1306_WHITE). */
static const uint16_t DISPLAY_COLOR_WHITE = 1;

#endif
