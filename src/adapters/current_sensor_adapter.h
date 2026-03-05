#ifndef CURRENT_SENSOR_ADAPTER_H
#define CURRENT_SENSOR_ADAPTER_H

/**
 * Current/voltage sensor interface (e.g. INA219). Provides bus voltage, current, power.
 */
class CurrentSensorAdapter {
public:
    virtual ~CurrentSensorAdapter() = default;

    virtual bool begin() = 0;
    virtual bool isConnected() const = 0;
    virtual float getBusVoltageV() = 0;
    virtual float getCurrentMa() = 0;
    virtual float getPowerMw() = 0;

protected:
    CurrentSensorAdapter() = default;
};

#endif
