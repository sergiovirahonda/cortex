#ifndef BAROMETER_ADAPTER_H
#define BAROMETER_ADAPTER_H

/**
 * Barometer interface. Implementations (e.g. BME280) provide pressure and altitude.
 */
class BarometerAdapter {
public:
    virtual ~BarometerAdapter() = default;

    virtual bool begin() = 0;
    virtual bool isConnected() const = 0;
    virtual float readPressureHpa() = 0;
    virtual float readAltitudeMeters(float seaLevelHpa = 1013.25f) = 0;

protected:
    BarometerAdapter() = default;
};

#endif
