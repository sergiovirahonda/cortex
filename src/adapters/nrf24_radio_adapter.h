#ifndef NRF24_RADIO_ADAPTER_H
#define NRF24_RADIO_ADAPTER_H

#include "radio_adapter.h"

class RF24;

/** nRF24L01 (RF24 library) implementation of RadioAdapter. */
class Nrf24RadioAdapter : public RadioAdapter {
public:
    Nrf24RadioAdapter(int cePin, int csnPin, byte* address);
    ~Nrf24RadioAdapter() override;

    void begin() override;
    bool isChipConnected() override;
    bool receivePacket(DronePacket& packet) override;
    void sendTelemetry(TelemetryData& data) override;

private:
    RF24* radio_;
    int cePin_;
    int csnPin_;
    byte* address_;
};

#endif
