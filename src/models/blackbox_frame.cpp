#include "blackbox_frame.h"

void BlackboxFrame::populate(
    const Attitude& attitude,
    const DroneCommand& command,
    const AvionicsMetrics& avionics,
    const MotorOutput& motors,
    uint32_t timestampMs)
{
    this->timestampMs = timestampMs;

    this->attitude.rollAngle = attitude.getRollAngle();
    this->attitude.rollRate = attitude.getRollRate();
    this->attitude.pitchAngle = attitude.getPitchAngle();
    this->attitude.pitchRate = attitude.getPitchRate();
    this->attitude.yawRate = attitude.getYawRate();
    this->attitude.headingDeg = attitude.getHeadingDeg();
    this->attitude.headingInitialized = attitude.getHeadingInitialized();
    this->attitude.onGround = attitude.getOnGround();
    this->attitude.rollErrorSum = attitude.getRollErrorSum();
    this->attitude.pitchErrorSum = attitude.getPitchErrorSum();
    this->attitude.yawErrorSum = attitude.getYawErrorSum();

    this->command.pitch = command.getPitch();
    this->command.roll = command.getRoll();
    this->command.yaw = command.getYaw();
    this->command.throttle = command.getThrottle();
    this->command.rollTrim = command.getRollTrim();
    this->command.pitchTrim = command.getPitchTrim();
    this->command.yawTrim = command.getYawTrim();
    this->command.trimReset = command.getTrimReset();
    this->command.altitudeHold = command.getAltitudeHold();

    this->altitude.relativeAltitudeCm = (float)avionics.lidar.getDistanceCm();
    this->altitude.absoluteAltitudeM = avionics.barometer.getAltitudeMeters();

    this->gps.latitude = avionics.gps.getLatitude();
    this->gps.longitude = avionics.gps.getLongitude();
    this->gps.satellites = avionics.gps.getSatellites();
    this->gps.valid = avionics.gps.getLocationIsValid();
    this->gps.altitudeMeters = avionics.gps.getAltitudeMeters();

    this->compass.azimuthDeg = (float)avionics.compass.getSmoothedAzimuth();
    this->compass.available = avionics.compass.getCompassHealth();

    this->motors.m1 = motors.getMotor1Speed();
    this->motors.m2 = motors.getMotor2Speed();
    this->motors.m3 = motors.getMotor3Speed();
    this->motors.m4 = motors.getMotor4Speed();
}
