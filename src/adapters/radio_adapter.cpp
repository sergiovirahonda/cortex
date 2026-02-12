#include "radio_adapter.h"
#include <SPI.h>

RadioAdapter::RadioAdapter(
    int cePin,
    int csnPin,
    byte* address
) {
    this->cePin = cePin;
    this->csnPin = csnPin;
    this->address = address;
    this->radio = RF24(cePin, csnPin);
}

void RadioAdapter::begin() {
    bool started = this->radio.begin();
    if (!started) {
        // Handle radio initialization failure
        Serial.println("Radio hardware not responding!");
        while (1);
    }
    Serial.println("Radio started!");
    // Explicitly disable variable sizing (Safety first)
    this->radio.disableDynamicPayloads();
    // Match TX configuration
    this->radio.setAutoAck(true);
    this->radio.enableAckPayload();
    this->radio.setPALevel(RF24_PA_LOW);  // Match TX side
    this->radio.setDataRate(RF24_250KBPS); // Longest Range
    this->radio.setChannel(108); // Avoid WiFi interference (Above 2.48GHz) 
    this->radio.openReadingPipe(1, this->address);
    this->radio.startListening();
}

bool RadioAdapter::receivePacket(DronePacket &packet) {
    if (this->radio.available()) {
        this->radio.read(&packet, sizeof(DronePacket));
        return true;
    } else {
        return false;
    }
}

void RadioAdapter::sendTelemetry(TelemetryData &data) {
    this->radio.writeAckPayload(1, &data, sizeof(TelemetryData));
}

bool RadioAdapter::isChipConnected() {
    return this->radio.isChipConnected();
}