#ifndef COMPASS_ADAPTER_H
#define COMPASS_ADAPTER_H

/**
 * Compass interface. Implementations provide azimuth (raw and optionally smoothed).
 */
class CompassAdapter {
public:
    virtual ~CompassAdapter() = default;

    virtual void begin() = 0;
    virtual void update() = 0;
    virtual int getRawAzimuth() = 0;
    virtual int getSmoothedAzimuth() = 0;

protected:
    CompassAdapter() = default;
};

#endif
