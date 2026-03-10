#include "blackbox_controller.h"
#include <stdio.h>
#include <cinttypes>

BlackboxController::BlackboxController(StorageAdapter* storage, const DroneConfig& config)
    : storage_(storage),
      config_(config),
      queue_(nullptr),
      nextLogIndex_(1),
      blackboxUnavailable_(true),
      droppedFrames_(0) {}

BlackboxController::~BlackboxController() {
    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
    if (storage_) {
        storage_->closeLog();
    }
}

void BlackboxController::init() {
    if (queue_ != nullptr) return;  // Already inited
    const UBaseType_t queueLen = (UBaseType_t)config_.getBlackboxQueueLen();
    if (queueLen == 0) {
        blackboxUnavailable_ = true;
        return;
    }
    queue_ = xQueueCreate(queueLen, sizeof(BlackboxFrame));
    if (queue_ == nullptr) {
        blackboxUnavailable_ = true;
        return;
    }

    if (!storage_ || !storage_->begin()) {
        blackboxUnavailable_ = true;
        return;
    }
    if (!storage_->isAvailable()) {
        blackboxUnavailable_ = true;
        return;
    }

    (void)storage_->readIndexFile(config_.getBlackboxIndexFilename(), nextLogIndex_);
    if (nextLogIndex_ < 1) nextLogIndex_ = 1;

    if (!openNextLog()) {
        blackboxUnavailable_ = true;
        return;
    }
    blackboxUnavailable_ = false;
}

bool BlackboxController::openNextLog() {
    char path[40];
    snprintf(path, sizeof(path), "%s%03u%s",
             config_.getBlackboxLogPrefix(),
             (unsigned)nextLogIndex_,
             config_.getBlackboxLogSuffix());

    if (!storage_->openLog(path)) return false;
    writeCsvHeader();
    (void)storage_->writeIndexFile(config_.getBlackboxIndexFilename(), nextLogIndex_ + 1);
    nextLogIndex_++;
    return true;
}

bool BlackboxController::tryEnqueue(const BlackboxFrame& frame) {
    if (queue_ == nullptr) return false;
    if (xQueueSendToBack(queue_, &frame, 0) != pdTRUE) {
        droppedFrames_++;
        return false;
    }
    return true;
}

void BlackboxController::writeCsvHeader() {
    const char* header =
        "timestamp_ms,batt_v,current_ma,"
        "roll_angle,roll_rate,pitch_angle,"
        "pitch_rate,yaw_rate,heading_deg,heading_init,"
        "on_ground,roll_error_sum,pitch_error_sum,"
        "yaw_error_sum,pitch,roll,yaw,throttle,roll_trim,"
        "pitch_trim,yaw_trim,trim_reset,altitude_hold,"
        "rel_alt_cm,abs_alt_m,gps_lat,gps_lng,gps_sats,"
        "gps_valid,gps_alt_m,compass_azimuth,compass_avail,"
        "m1,m2,m3,m4";
    (void)storage_->writeLine(header);
}

void BlackboxController::formatFrameToCsv(const BlackboxFrame& frame, char* buf, size_t bufSize) {
    snprintf(buf, bufSize,
        "%" PRIu32 ",%.2f,%.2f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,"
        "%.4f,%.4f,%.4f,"
        "%d,%d,%d,%d,%d,%d,%d,%u,%d,"
        "%.2f,%.2f,"
        "%.6f,%.6f,%" PRIu32 ",%d,%.2f,"
        "%.2f,%d,"
        "%d,%d,%d,%d",
        frame.timestampMs,
        frame.power.battVoltage, frame.power.current_mA,
        frame.attitude.rollAngle, frame.attitude.rollRate,
        frame.attitude.pitchAngle, frame.attitude.pitchRate,
        frame.attitude.yawRate, frame.attitude.headingDeg,
        frame.attitude.headingInitialized ? 1 : 0, frame.attitude.onGround ? 1 : 0,
        frame.attitude.rollErrorSum, frame.attitude.pitchErrorSum, frame.attitude.yawErrorSum,
        frame.command.pitch, frame.command.roll, frame.command.yaw, frame.command.throttle,
        frame.command.rollTrim, frame.command.pitchTrim, frame.command.yawTrim,
        (unsigned)frame.command.trimReset, (int)frame.command.altitudeHold,
        frame.altitude.relativeAltitudeCm, frame.altitude.absoluteAltitudeM,
        frame.gps.latitude, frame.gps.longitude, frame.gps.satellites,
        frame.gps.valid ? 1 : 0, frame.gps.altitudeMeters,
        frame.compass.azimuthDeg, frame.compass.available ? 1 : 0,
        frame.motors.m1, frame.motors.m2, frame.motors.m3, frame.motors.m4);
}

void BlackboxController::processNextFrame() {
    if (queue_ == nullptr) return;

    BlackboxFrame frame;
    if (xQueueReceive(queue_, &frame, 0) != pdTRUE) return;

    if (blackboxUnavailable_) return;  // Drain only

    static char lineBuf[512];
    formatFrameToCsv(frame, lineBuf, sizeof(lineBuf));
    if (!storage_->writeLine(lineBuf)) {
        blackboxUnavailable_ = true;
        storage_->closeLog();
    }
}
