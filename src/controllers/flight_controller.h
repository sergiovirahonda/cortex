#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../models/attitude.h"
#include "../models/motor_output.h"
#include "../models/drone_command.h"
#include "../config/drone_config.h"

class FlightController {
    private:
        // Failsafe state
        unsigned long lastPacketTime;
        bool inFailsafe;
        int16_t failsafeThrottle;
        
        // Trim debounce state
        unsigned long lastTrimTime;
        
        // Failsafe constants
        static const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
        static const int16_t FAILSAFE_LANDING_THROTTLE = 1200;
        static const int16_t FAILSAFE_THROTTLE_DECAY = 5;
        
    public:
        FlightController();
        void updateFailsafe(bool packetReceived, int16_t currentThrottle);
        bool isInFailsafe() const;
        void applyFailsafe(DroneCommand& command);
        void updateTrims(DroneCommand& command, AttitudeTrim& attitudeTrim, const DroneConfig& droneConfig);
        void computeAttitudeCorrections(
            const DroneConfig& droneConfig,
            DroneCommand& command,
            Attitude& attitude,
            AttitudeTrim& trim,
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