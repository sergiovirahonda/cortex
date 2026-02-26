#include "tf_luna_adapter.h"

TFLunaLidarAdapter::TFLunaLidarAdapter(HardwareSerial* serial)
    : serial_(serial), distanceCm_(0), signalStrength_(0), temperatureC_(0),
      distBufIdx_(0), distBufCount_(0), distSum_(0) {}

void TFLunaLidarAdapter::begin() {
    distBufIdx_ = 0;
    distBufCount_ = 0;
    distSum_ = 0;
}

void TFLunaLidarAdapter::update() {
    if (!serial_ || serial_->available() < FRAME_LEN) return;

    while (serial_->available() >= FRAME_LEN) {
        if (serial_->peek() != HEADER0) {
            serial_->read();
            continue;
        }
        uint8_t buf[FRAME_LEN];
        for (int i = 0; i < FRAME_LEN; i++) buf[i] = serial_->read();
        if (buf[1] != HEADER1) continue;

        uint8_t sum = 0;
        for (int i = 0; i < FRAME_LEN - 1; i++) sum += buf[i];
        if ((sum & 0xFF) != buf[FRAME_LEN - 1]) continue;

        uint16_t rawDist = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        signalStrength_  = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        int16_t t        = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
        temperatureC_    = (t / 8.0f) - 256.0f;

        // Moving average on distance
        if (distBufCount_ < DIST_FILTER_SIZE) {
            distSum_ += rawDist;
            distBuf_[distBufIdx_] = rawDist;
            distBufIdx_ = (distBufIdx_ + 1) % DIST_FILTER_SIZE;
            distBufCount_++;
        } else {
            distSum_ = distSum_ - distBuf_[distBufIdx_] + rawDist;
            distBuf_[distBufIdx_] = rawDist;
            distBufIdx_ = (distBufIdx_ + 1) % DIST_FILTER_SIZE;
        }
        distanceCm_ = (uint16_t)(distSum_ / distBufCount_);
    }
}
