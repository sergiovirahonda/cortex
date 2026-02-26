#include "nrf24_radio_adapter.h"
#include <RF24.h>
#include <SPI.h>

Nrf24RadioAdapter::Nrf24RadioAdapter(int cePin, int csnPin, byte* address)
    : cePin_(cePin), csnPin_(csnPin), address_(address) {
    radio_ = new RF24(cePin, csnPin);
}

Nrf24RadioAdapter::~Nrf24RadioAdapter() {
    delete radio_;
    radio_ = nullptr;
}

void Nrf24RadioAdapter::begin() {
    bool started = radio_->begin();
    if (!started) {
        Serial.println("Radio hardware not responding!");
        while (1);
    }
    Serial.println("Radio started!");
    radio_->disableDynamicPayloads();
    radio_->setAutoAck(true);
    radio_->enableAckPayload();
    radio_->setPALevel(RF24_PA_HIGH);
    radio_->setDataRate(RF24_250KBPS);
    radio_->setChannel(108);
    radio_->openReadingPipe(1, address_);
    radio_->startListening();
}

bool Nrf24RadioAdapter::isChipConnected() {
    return radio_->isChipConnected();
}

bool Nrf24RadioAdapter::receivePacket(DronePacket& packet) {
    if (radio_->available()) {
        radio_->read(&packet, sizeof(DronePacket));
        return true;
    }
    return false;
}

void Nrf24RadioAdapter::sendTelemetry(TelemetryData& data) {
    radio_->writeAckPayload(1, &data, sizeof(TelemetryData));
}
