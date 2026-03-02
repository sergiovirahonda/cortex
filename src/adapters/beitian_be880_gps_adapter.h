#ifndef BEITIAN_BE880_GPS_ADAPTER_H
#define BEITIAN_BE880_GPS_ADAPTER_H

#include "gps_adapter.h"

class TinyGPSPlus;

/** Beitian BE880 GPS: NMEA over UART, implemented via TinyGPS+. */
class BeitianBe880GpsAdapter : public GpsAdapter {
public:
    BeitianBe880GpsAdapter(HardwareSerial* serial, uint32_t baud, int rxPin, int txPin);
    ~BeitianBe880GpsAdapter() override;

    void begin() override;
    void update() override;

    bool locationIsValid() const override;
    double getLat() override;
    double getLng() override;
    uint32_t getSatellites() override;
    bool altitudeIsValid() const override;
    double getAltitudeMeters() override;

    uint32_t getCharsProcessed() override;
    uint32_t getPassedChecksum() override;
    uint32_t getSentencesWithFix() override;

private:
    HardwareSerial* serial_;
    uint32_t baud_;
    int rxPin_;
    int txPin_;
    TinyGPSPlus* gps_;
};

#endif
