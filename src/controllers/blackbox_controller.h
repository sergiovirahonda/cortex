#ifndef BLACKBOX_CONTROLLER_H
#define BLACKBOX_CONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "../models/blackbox_frame.h"
#include "../adapters/storage_adapter.h"
#include "../config/drone_config.h"

/** Manages blackbox logging: queue, SD file, incremental naming, and frame writing. */
class BlackboxController {
public:
    /** Process rate in ms (used by blackbox task for delay). */
    uint32_t getProcessRateMs() const { return config_.getBlackboxProcessRateMs(); }

    explicit BlackboxController(StorageAdapter* storage, const DroneConfig& config);
    ~BlackboxController();

    /** Create queue, init storage, read index, open first log and write header. Sets blackboxUnavailable on failure. */
    void init();

    QueueHandle_t getQueueHandle() const { return queue_; }
    bool isBlackboxUnavailable() const { return blackboxUnavailable_; }
    uint32_t getDroppedFramesCount() const { return droppedFrames_; }

    /** Non-blocking enqueue from flight loop. Returns false and increments dropped count when queue full. */
    bool tryEnqueue(const BlackboxFrame& frame);

    /** Call from blackbox task only: receive one frame, format and write to SD. Drains queue when unavailable. */
    void processNextFrame();

private:
    StorageAdapter* storage_;
    const DroneConfig& config_;
    QueueHandle_t queue_;
    uint16_t nextLogIndex_;
    bool blackboxUnavailable_;
    uint32_t droppedFrames_;

    bool openNextLog();
    void writeCsvHeader();
    void formatFrameToCsv(const BlackboxFrame& frame, char* buf, size_t bufSize);
};

#endif
