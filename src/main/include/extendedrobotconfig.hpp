#pragma once

#include <wpi/json.h>
#include "pathplanner/lib/config/RobotConfig.h"

namespace pathplanner
{
    class ExtendedRobotConfig : public RobotConfig
    {
        public:
            static RobotConfig from_json(wpi::json json);
            static frc::DCMotor getMotorFromSettingsString(std::string motorStr,
                    int numMotors);
    };
}