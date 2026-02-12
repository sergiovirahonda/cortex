#ifndef RADIO_ADAPTER_H
#define RADIO_ADAPTER_H

#include <nRF24L01.h>
#include <RF24.h>
#include "../models/drone_command.h"

class RadioAdapter {
    private:
        RF24 radio;
        int cePin;
        int csnPin;
        byte* address;
    
    public:
        RadioAdapter(
            int cePin,
            int csnPin,
            byte* address
        );
        void begin();
        bool isChipConnected();
        bool receivePacket(DronePacket &packet);
        void sendTelemetry(TelemetryData &data);
};

#endif


