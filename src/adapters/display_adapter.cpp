#include "display_adapter.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

DisplayAdapter::DisplayAdapter(
    int width,
    int height,
    TwoWire* wire,
    int rstPin,
    uint8_t i2cAddr
) {
    width_ = width;
    height_ = height;
    wire_ = wire;
    rstPin_ = rstPin;
    i2cAddr_ = i2cAddr;
    display_ = new Adafruit_SSD1306(
        (uint8_t)width,
        (uint8_t)height,
        wire,
        (int8_t)rstPin
    );
}

DisplayAdapter::~DisplayAdapter() {
    delete display_;
    display_ = nullptr;
}

bool DisplayAdapter::begin() {
    return display_->begin(SSD1306_SWITCHCAPVCC, i2cAddr_);
}

void DisplayAdapter::clearDisplay() {
    display_->clearDisplay();
}

void DisplayAdapter::setCursor(int16_t x, int16_t y) {
    display_->setCursor(x, y);
}

void DisplayAdapter::display() {
    display_->display();
}

void DisplayAdapter::setTextSize(uint8_t s) {
    display_->setTextSize(s);
}

void DisplayAdapter::setTextColor(uint16_t color) {
    display_->setTextColor(color);
}

void DisplayAdapter::invertDisplay(bool i) {
    display_->invertDisplay(i);
}

size_t DisplayAdapter::write(uint8_t c) {
    return display_->write(c);
}
