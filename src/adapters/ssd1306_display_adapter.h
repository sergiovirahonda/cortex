#ifndef SSD1306_DISPLAY_ADAPTER_H
#define SSD1306_DISPLAY_ADAPTER_H

#include "display_adapter.h"
#include <Wire.h>

class Adafruit_SSD1306;

/** SSD1306 OLED (Adafruit driver) implementation of DisplayAdapter. */
class Ssd1306DisplayAdapter : public DisplayAdapter {
public:
    Ssd1306DisplayAdapter(
        int width,
        int height,
        TwoWire* wire,
        int rstPin,
        uint8_t i2cAddr = 0x3C
    );
    ~Ssd1306DisplayAdapter() override;

    bool begin() override;
    void clearDisplay() override;
    void setCursor(int16_t x, int16_t y) override;
    void display() override;
    void setTextSize(uint8_t s) override;
    void setTextColor(uint16_t color) override;
    void invertDisplay(bool i) override;
    size_t write(uint8_t c) override;

private:
    Adafruit_SSD1306* display_;
    int width_;
    int height_;
    TwoWire* wire_;
    int rstPin_;
    uint8_t i2cAddr_;
};

#endif
