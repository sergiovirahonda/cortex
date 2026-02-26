#ifndef DISPLAY_ADAPTER_H
#define DISPLAY_ADAPTER_H

#include <Arduino.h>

/**
 * Display interface (e.g. OLED). Inherits Print so print()/println() work.
 * Implementations wrap a specific display driver (e.g. SSD1306).
 */
class DisplayAdapter : public Print {
public:
    virtual ~DisplayAdapter() = default;

    /** Initialize display; returns false on failure. */
    virtual bool begin() = 0;

    virtual void clearDisplay() = 0;
    virtual void setCursor(int16_t x, int16_t y) = 0;
    virtual void display() = 0;
    virtual void setTextSize(uint8_t s) = 0;
    virtual void setTextColor(uint16_t color) = 0;
    virtual void invertDisplay(bool i) = 0;

    /** Print interface: forward to underlying display. */
    size_t write(uint8_t c) override = 0;

protected:
    DisplayAdapter() = default;
};

/** Common color for setTextColor (matches SSD1306_WHITE). */
static const uint16_t DISPLAY_COLOR_WHITE = 1;

#endif
