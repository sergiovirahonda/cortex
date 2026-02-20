#include "gps_adapter.h"

GpsAdapter::GpsAdapter(HardwareSerial* serial, uint32_t baud, int rxPin, int txPin)
    : serial_(serial), baud_(baud), rxPin_(rxPin), txPin_(txPin) {}

void GpsAdapter::begin() {
    if (serial_) {
        serial_->begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    }
}

void GpsAdapter::update() {
    if (!serial_) return;
    while (serial_->available() > 0) {
        gps_.encode(serial_->read());
    }
}

bool GpsAdapter::locationIsValid() const {
    return gps_.location.isValid();
}

double GpsAdapter::getLat() {
    return gps_.location.lat();
}

double GpsAdapter::getLng() {
    return gps_.location.lng();
}

uint32_t GpsAdapter::getSatellites() {
    return gps_.satellites.value();
}

bool GpsAdapter::altitudeIsValid() const {
    return gps_.altitude.isValid();
}

double GpsAdapter::getAltitudeMeters() {
    return gps_.altitude.meters();
}

uint32_t GpsAdapter::getCharsProcessed() {
    return gps_.charsProcessed();
}

uint32_t GpsAdapter::getPassedChecksum() {
    return gps_.passedChecksum();
}

uint32_t GpsAdapter::getSentencesWithFix() {
    return gps_.sentencesWithFix();
}
