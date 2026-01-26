#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../models/attitude.h"
#include "../models/motor_output.h"
#include "../models/drone_command.h"

class FlightController {
    public:
        FlightController();
        void computeMotorOutput(
            MotorOutput& motorOutput,
            DroneCommand& command,
            float rollPID,
            float pitchPID,
            float yawPID
        );
};

#endif