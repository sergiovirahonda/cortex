#ifndef SD_CARD_STORAGE_ADAPTER_H
#define SD_CARD_STORAGE_ADAPTER_H

#include "storage_adapter.h"

struct SdCardStorageAdapterImpl;

class SdCardStorageAdapter : public StorageAdapter {
public:
    SdCardStorageAdapter(uint8_t csPin, int mosiPin, int sckPin, int misoPin);
    ~SdCardStorageAdapter() override;

    bool begin() override;
    bool isAvailable() const override;
    bool openLog(const char* path) override;
    bool writeLine(const char* line) override;
    void closeLog() override;
    bool readIndexFile(const char* path, uint16_t& outIndex) override;
    bool writeIndexFile(const char* path, uint16_t index) override;

private:
    uint8_t csPin_;
    int mosiPin_;
    int sckPin_;
    int misoPin_;
    bool available_;
    SdCardStorageAdapterImpl* impl_;
};

#endif
