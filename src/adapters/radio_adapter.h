#ifndef RADIO_ADAPTER_H
#define RADIO_ADAPTER_H

#include "../models/drone_command.h"

/**
 * Radio interface for receiving commands and sending telemetry (e.g. nRF24L01).
 */
class RadioAdapter {
public:
    virtual ~RadioAdapter() = default;

    virtual void begin() = 0;
    virtual bool isChipConnected() = 0;
    virtual bool receivePacket(DronePacket& packet) = 0;
    virtual void sendTelemetry(TelemetryData& data) = 0;

protected:
    RadioAdapter() = default;
};

#endif
