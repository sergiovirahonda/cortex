#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../models/attitude.h"
#include "../models/motor_output.h"
#include "../models/drone_command.h"

class FlightController {
    private:
        // Failsafe state
        unsigned long lastPacketTime;
        bool inFailsafe;
        int16_t failsafeThrottle;
        
        // Failsafe constants
        static const unsigned long FAILSAFE_TIMEOUT_MS = 500;
        static const int16_t FAILSAFE_LANDING_THROTTLE = 1100;
        static const int16_t FAILSAFE_THROTTLE_DECAY = 5;
        
    public:
        FlightController();
        void updateFailsafe(bool packetReceived, int16_t currentThrottle);
        bool isInFailsafe() const;
        void applyFailsafe(DroneCommand& command);
        void computeAttitudeCorrections(
            DroneCommand& command,
            Attitude& attitude,
            float& pitchPD,
            float& rollPD,
            float& yawPD
        );
        void computeMotorOutput(
            MotorOutput& motorOutput,
            DroneCommand& command,
            AttitudeTrim& trim,
            float rollPID,
            float pitchPID,
            float yawPID
        );
};

#endif