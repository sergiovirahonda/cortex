#include "sd_card_storage_adapter.h"
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

struct SdCardStorageAdapterImpl {
    SPIClass* spi = nullptr;
    File logFile;
};

SdCardStorageAdapter::SdCardStorageAdapter(uint8_t csPin, int mosiPin, int sckPin, int misoPin)
    : csPin_(csPin),
      mosiPin_(mosiPin),
      sckPin_(sckPin),
      misoPin_(misoPin),
      available_(false),
      impl_(nullptr) {
    impl_ = new SdCardStorageAdapterImpl();
    impl_->spi = new SPIClass(2);  // SPI2 to avoid conflict with other peripherals
}

SdCardStorageAdapter::~SdCardStorageAdapter() {
    closeLog();
    if (impl_) {
        if (impl_->spi) {
            impl_->spi->end();
            delete impl_->spi;
        }
        delete impl_;
        impl_ = nullptr;
    }
}

bool SdCardStorageAdapter::begin() {
    if (!impl_ || !impl_->spi) return false;

    // Custom SPI pins: SCK, MISO, MOSI (CS is separate)
    impl_->spi->begin(sckPin_, misoPin_, mosiPin_, -1);

    const uint32_t sdFreqHz = 4000000;
    available_ = SD.begin(csPin_, *impl_->spi, sdFreqHz);

    return available_;
}

bool SdCardStorageAdapter::isAvailable() const {
    return available_;
}

bool SdCardStorageAdapter::openLog(const char* path) {
    if (!available_ || !impl_) return false;
    closeLog();
    if (SD.exists(path)) {
        SD.remove(path);
    }
    impl_->logFile = SD.open(path, FILE_WRITE);
    return impl_->logFile;
}

bool SdCardStorageAdapter::writeLine(const char* line) {
    if (!impl_ || !impl_->logFile) return false;
    impl_->logFile.println(line);
    return true;
}

void SdCardStorageAdapter::closeLog() {
    if (impl_ && impl_->logFile) {
        impl_->logFile.close();
    }
}

bool SdCardStorageAdapter::readIndexFile(const char* path, uint16_t& outIndex) {
    if (!available_) return false;
    if (!SD.exists(path)) {
        outIndex = 1;
        return true;  // First run: use 1
    }
    File f = SD.open(path, FILE_READ);
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
    if (SD.exists(path)) {
        SD.remove(path);
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    f.println(index);
    f.close();
    return true;
}
