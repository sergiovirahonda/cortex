#ifndef SD_CARD_STORAGE_ADAPTER_H
#define SD_CARD_STORAGE_ADAPTER_H

#include "storage_adapter.h"

struct SdCardStorageAdapterImpl;

class SdCardStorageAdapter : public StorageAdapter {
public:
    SdCardStorageAdapter(int clkPin, int cmdPin, int d0Pin);
    ~SdCardStorageAdapter() override;

    bool begin() override;
    bool isAvailable() const override;
    bool openLog(const char* path) override;
    bool writeLine(const char* line) override;
    void closeLog() override;
    bool readIndexFile(const char* path, uint16_t& outIndex) override;
    bool writeIndexFile(const char* path, uint16_t index) override;

private:
    int clkPin_;
    int cmdPin_;
    int d0Pin_;
    bool available_;
    SdCardStorageAdapterImpl* impl_;
};

#endif
