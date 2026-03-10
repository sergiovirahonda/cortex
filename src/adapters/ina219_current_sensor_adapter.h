#ifndef INA219_CURRENT_SENSOR_ADAPTER_H
#define INA219_CURRENT_SENSOR_ADAPTER_H

#include "current_sensor_adapter.h"
#include <Wire.h>

class Adafruit_INA219;

/** INA219 implementation of CurrentSensorAdapter (I2C, same bus as BME280/compass). */
class Ina219CurrentSensorAdapter : public CurrentSensorAdapter {
public:
    explicit Ina219CurrentSensorAdapter(TwoWire* wire);
    ~Ina219CurrentSensorAdapter() override;

    bool begin() override;
    bool isConnected() const override;
    float getBusVoltageV() override;
    float getCurrentMa() override;
    float getPowerMw() override;

private:
    Adafruit_INA219* ina_;
    TwoWire* wire_;
    bool connected_;
};

#endif
