#ifndef GPS_ADAPTER_H
#define GPS_ADAPTER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

/**
 * GPS over UART (NMEA). Pass serial and baud/pins; call begin(), then update() each loop, then use getters.
 */
class GpsAdapter {
public:
    GpsAdapter(HardwareSerial* serial, uint32_t baud, int rxPin, int txPin);
    void begin();
    void update();

    bool locationIsValid() const;
    double getLat();
    double getLng();
    uint32_t getSatellites();
    bool altitudeIsValid() const;
    double getAltitudeMeters();

    uint32_t getCharsProcessed();
    uint32_t getPassedChecksum();
    uint32_t getSentencesWithFix();

private:
    HardwareSerial* serial_;
    uint32_t baud_;
    int rxPin_;
    int txPin_;
    TinyGPSPlus gps_;
};

#endif
