#include "beitian_be880_gps_adapter.h"
#include <TinyGPSPlus.h>

BeitianBe880GpsAdapter::BeitianBe880GpsAdapter(HardwareSerial* serial, uint32_t baud, int rxPin, int txPin)
    : serial_(serial), baud_(baud), rxPin_(rxPin), txPin_(txPin) {
    gps_ = new TinyGPSPlus();
}

BeitianBe880GpsAdapter::~BeitianBe880GpsAdapter() {
    delete gps_;
    gps_ = nullptr;
}

void BeitianBe880GpsAdapter::begin() {
    if (serial_) {
        serial_->begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    }
}

void BeitianBe880GpsAdapter::update() {
    if (!serial_) return;
    while (serial_->available() > 0) {
        gps_->encode(serial_->read());
    }
}

bool BeitianBe880GpsAdapter::locationIsValid() const {
    return gps_->location.isValid();
}

double BeitianBe880GpsAdapter::getLat() {
    return gps_->location.lat();
}

double BeitianBe880GpsAdapter::getLng() {
    return gps_->location.lng();
}

uint32_t BeitianBe880GpsAdapter::getSatellites() {
    return gps_->satellites.value();
}

bool BeitianBe880GpsAdapter::altitudeIsValid() const {
    return gps_->altitude.isValid();
}

double BeitianBe880GpsAdapter::getAltitudeMeters() {
    return gps_->altitude.meters();
}

uint32_t BeitianBe880GpsAdapter::getCharsProcessed() {
    return gps_->charsProcessed();
}

uint32_t BeitianBe880GpsAdapter::getPassedChecksum() {
    return gps_->passedChecksum();
}

uint32_t BeitianBe880GpsAdapter::getSentencesWithFix() {
    return gps_->sentencesWithFix();
}
