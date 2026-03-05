#ifndef INA219_READINGS_H
#define INA219_READINGS_H

class Ina219Readings {
public:
    Ina219Readings();

    float getBusVoltageV() const;
    float getCurrentMa() const;
    float getPowerMw() const;
    bool isConnected() const;

    void setBusVoltageV(float v);
    void setCurrentMa(float ma);
    void setPowerMw(float mw);
    void setConnected(bool connected);

private:
    float busVoltageV_;
    float currentMa_;
    float powerMw_;
    bool connected_;
};

#endif
