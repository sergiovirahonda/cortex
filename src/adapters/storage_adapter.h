#ifndef STORAGE_ADAPTER_H
#define STORAGE_ADAPTER_H

#include <Arduino.h>

/**
 * Storage interface for blackbox logging (e.g. SD card).
 * Inject into BlackboxController so I/O is testable and swappable.
 */
class StorageAdapter {
public:
    virtual ~StorageAdapter() = default;

    /** Initialize storage (e.g. mount SD). Return true if ready. */
    virtual bool begin() = 0;
    /** True if storage is available for read/write. */
    virtual bool isAvailable() const = 0;

    /** Open a new log file at path; truncate if exists. Return true if open for writing. */
    virtual bool openLog(const char* path) = 0;
    /** Write a line (no newline added). Return true on success. */
    virtual bool writeLine(const char* line) = 0;
    /** Close the current log file. */
    virtual void closeLog() = 0;

    /** Read first line as decimal index into outIndex. Return false if missing or invalid. */
    virtual bool readIndexFile(const char* path, uint16_t& outIndex) = 0;
    /** Write index as single line. Return true on success. */
    virtual bool writeIndexFile(const char* path, uint16_t index) = 0;

protected:
    StorageAdapter() = default;
};

#endif
