#ifndef BAROMETER_READINGS_H
#define BAROMETER_READINGS_H

class BarometerReadings {
public:
    BarometerReadings();

    float getPressureHpa() const;
    float getAltitudeMeters() const;
    bool isConnected() const;

    void setPressureHpa(float hpa);
    void setAltitudeMeters(float meters);
    void setConnected(bool connected);

private:
    float pressureHpa_;
    float altitudeMeters_;
    bool connected_;
};

#endif
