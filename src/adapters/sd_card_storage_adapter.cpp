#include "sd_card_storage_adapter.h"
#include <SD_MMC.h>
#include <FS.h>
#include <Arduino.h>

struct SdCardStorageAdapterImpl {
    File logFile;
};

static const char* SDMMC_MOUNT_POINT = "/sdcard";

static void makeFullPath(char* out, size_t outSize, const char* path) {
    // 1. If the config accidentally passed "/sdcard/...", strip it out completely.
    if (strncmp(path, "/sdcard", 7) == 0) {
        path += 7; // Move the pointer past the exact phrase "/sdcard"
    }
    
    // 2. Arduino SD_MMC wrapper handles the mount point automatically.
    // We only need to guarantee a single leading slash.
    if (path[0] == '/') {
        snprintf(out, outSize, "%s", path);
    } else {
        snprintf(out, outSize, "/%s", path);
    }
}

SdCardStorageAdapter::SdCardStorageAdapter(int clkPin, int cmdPin, int d0Pin)
    : clkPin_(clkPin),
      cmdPin_(cmdPin),
      d0Pin_(d0Pin),
      available_(false),
      impl_(nullptr) {
    impl_ = new SdCardStorageAdapterImpl();
}

SdCardStorageAdapter::~SdCardStorageAdapter() {
    closeLog();
    if (impl_) {
        delete impl_;
        impl_ = nullptr;
    }
    if (available_) {
        SD_MMC.end();
        available_ = false;
    }
}

bool SdCardStorageAdapter::begin() {
    if (!impl_) return false;

    // Software pull-ups backing up the physical 10k resistors
    pinMode(cmdPin_, INPUT_PULLUP);
    pinMode(d0Pin_, INPUT_PULLUP);

    if (!SD_MMC.setPins((int8_t)clkPin_, (int8_t)cmdPin_, (int8_t)d0Pin_)) {
        return false;
    }

    if (!SD_MMC.begin(SDMMC_MOUNT_POINT, true)) { // true = 1-bit mode
        return false;
    }

    if (SD_MMC.cardType() == CARD_NONE) {
        SD_MMC.end();
        return false;
    }

    available_ = true;
    return true;
}

bool SdCardStorageAdapter::isAvailable() const {
    return available_;
}

bool SdCardStorageAdapter::openLog(const char* path) {
    if (!available_ || !impl_) return false;
    closeLog();
    char fullPath[64];
    makeFullPath(fullPath, sizeof(fullPath), path);

    // If the drone reboots mid-air, we keep the pre-crash data.
    impl_->logFile = SD_MMC.open(fullPath, FILE_APPEND);
    
    return impl_->logFile;
}

bool SdCardStorageAdapter::writeLine(const char* line) {
    if (!impl_ || !impl_->logFile) return false;
    
    impl_->logFile.println(line);
    impl_->logFile.flush(); // Survive instant power loss
    
    return true;
}

void SdCardStorageAdapter::closeLog() {
    if (impl_ && impl_->logFile) {
        impl_->logFile.close();
    }
}

bool SdCardStorageAdapter::readIndexFile(const char* path, uint16_t& outIndex) {
    if (!available_) return false;
    char fullPath[64];
    makeFullPath(fullPath, sizeof(fullPath), path);
    if (!SD_MMC.exists(fullPath)) {
        outIndex = 1;
        return true;
    }
    File f = SD_MMC.open(fullPath, FILE_READ);
    if (!f) return false;
    String line = f.readStringUntil('\n');
    f.close();
    line.trim();
    int n = line.toInt();
    if (n <= 0 || n > 65535) return false;
    outIndex = static_cast<uint16_t>(n);
    return true;
}

bool SdCardStorageAdapter::writeIndexFile(const char* path, uint16_t index) {
    if (!available_) return false;
    char fullPath[64];
    makeFullPath(fullPath, sizeof(fullPath), path);
    if (SD_MMC.exists(fullPath)) {
        SD_MMC.remove(fullPath);
    }
    File f = SD_MMC.open(fullPath, FILE_WRITE);
    if (!f) return false;
    f.println(index);
    f.close();
    return true;
}
