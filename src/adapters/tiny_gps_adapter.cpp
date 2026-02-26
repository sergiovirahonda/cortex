#include "tiny_gps_adapter.h"
#include <TinyGPSPlus.h>

TinyGpsAdapter::TinyGpsAdapter(HardwareSerial* serial, uint32_t baud, int rxPin, int txPin)
    : serial_(serial), baud_(baud), rxPin_(rxPin), txPin_(txPin) {
    gps_ = new TinyGPSPlus();
}

TinyGpsAdapter::~TinyGpsAdapter() {
    delete gps_;
    gps_ = nullptr;
}

void TinyGpsAdapter::begin() {
    if (serial_) {
        serial_->begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    }
}

void TinyGpsAdapter::update() {
    if (!serial_) return;
    while (serial_->available() > 0) {
        gps_->encode(serial_->read());
    }
}

bool TinyGpsAdapter::locationIsValid() const {
    return gps_->location.isValid();
}

double TinyGpsAdapter::getLat() {
    return gps_->location.lat();
}

double TinyGpsAdapter::getLng() {
    return gps_->location.lng();
}

uint32_t TinyGpsAdapter::getSatellites() {
    return gps_->satellites.value();
}

bool TinyGpsAdapter::altitudeIsValid() const {
    return gps_->altitude.isValid();
}

double TinyGpsAdapter::getAltitudeMeters() {
    return gps_->altitude.meters();
}

uint32_t TinyGpsAdapter::getCharsProcessed() {
    return gps_->charsProcessed();
}

uint32_t TinyGpsAdapter::getPassedChecksum() {
    return gps_->passedChecksum();
}

uint32_t TinyGpsAdapter::getSentencesWithFix() {
    return gps_->sentencesWithFix();
}
