#ifndef GPS_ADAPTER_H
#define GPS_ADAPTER_H

#include <Arduino.h>

/**
 * GPS interface (e.g. NMEA over UART). Call begin(), then update() each loop, then use getters.
 */
class GpsAdapter {
public:
    virtual ~GpsAdapter() = default;

    virtual void begin() = 0;
    virtual void update() = 0;

    virtual bool locationIsValid() const = 0;
    virtual double getLat() = 0;
    virtual double getLng() = 0;
    virtual uint32_t getSatellites() = 0;
    virtual bool altitudeIsValid() const = 0;
    virtual double getAltitudeMeters() = 0;

    virtual uint32_t getCharsProcessed() = 0;
    virtual uint32_t getPassedChecksum() = 0;
    virtual uint32_t getSentencesWithFix() = 0;

protected:
    GpsAdapter() = default;
};

#endif
