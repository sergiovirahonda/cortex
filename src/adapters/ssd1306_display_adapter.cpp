#include "ssd1306_display_adapter.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

Ssd1306DisplayAdapter::Ssd1306DisplayAdapter(
    int width,
    int height,
    TwoWire* wire,
    int rstPin,
    uint8_t i2cAddr
)
    : width_(width), height_(height), wire_(wire), rstPin_(rstPin), i2cAddr_(i2cAddr) {
    display_ = new Adafruit_SSD1306(
        (uint8_t)width,
        (uint8_t)height,
        wire,
        (int8_t)rstPin
    );
}

Ssd1306DisplayAdapter::~Ssd1306DisplayAdapter() {
    delete display_;
    display_ = nullptr;
}

bool Ssd1306DisplayAdapter::begin() {
    return display_->begin(SSD1306_SWITCHCAPVCC, i2cAddr_);
}

void Ssd1306DisplayAdapter::clearDisplay() {
    display_->clearDisplay();
}

void Ssd1306DisplayAdapter::setCursor(int16_t x, int16_t y) {
    display_->setCursor(x, y);
}

void Ssd1306DisplayAdapter::display() {
    display_->display();
}

void Ssd1306DisplayAdapter::setTextSize(uint8_t s) {
    display_->setTextSize(s);
}

void Ssd1306DisplayAdapter::setTextColor(uint16_t color) {
    display_->setTextColor(color);
}

void Ssd1306DisplayAdapter::invertDisplay(bool i) {
    display_->invertDisplay(i);
}

size_t Ssd1306DisplayAdapter::write(uint8_t c) {
    return display_->write(c);
}
