// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <memory>
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"
#include "frc2/command/Command.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "pathplanner/lib/commands/FollowPathCommand.h"

#include <fstream>
#include "Drivetrain.h"
#include "extendedrobotconfig.hpp"
#include <frc/Filesystem.h>
#include <sstream>

using namespace pathplanner;

std::shared_ptr<PathPlannerPath> create_path_from_json()
{
    std::ifstream f(frc::filesystem::GetDeployDirectory() + "/EvilPath.path" );
    wpi::json trajectory = wpi::json::parse(f);
    
    std::vector<Waypoint> waypoints;
    for (size_t i = 0; i < trajectory["waypoints"].size(); i++)
    {
        waypoints.push_back(Waypoint::fromJson(trajectory["waypoints"][i]));
    }

    std::vector<RotationTarget> rotationTargets;
    for (size_t i = 0; i < trajectory["rotationTargets"].size(); i++)
    {
        rotationTargets.push_back(RotationTarget::fromJson(trajectory["rotationTargets"][i]));
    }

    std::vector<PointTowardsZone> pointTowardsZones;
    for (size_t i = 0; i < trajectory["pointTowardsZones"].size(); i++)
    {
        pointTowardsZones.push_back(PointTowardsZone::fromJson(trajectory["pointTowardsZones"][i]));
    }

    std::vector<ConstraintsZone> constraintZones;
    for (size_t i = 0; i < trajectory["constraintZones"].size(); i++)
    {
        constraintZones.push_back(ConstraintsZone::fromJson(trajectory["constraintZones"][i]));
    }

    std::vector<EventMarker> eventMarkers;
    for (size_t i = 0; i < trajectory["eventMarkers"].size(); i++)
    {
        eventMarkers.push_back(EventMarker::fromJson(trajectory["eventMarkers"][i]));
    }

    PathConstraints globalConstraints = PathConstraints::fromJson(trajectory["globalConstraints"]);

    IdealStartingState idealStartingState = IdealStartingState::fromJson(trajectory["idealStartingState"]);

    GoalEndState goalEndState = GoalEndState::fromJson(trajectory["goalEndState"]);

    bool reversed = (trajectory)["reversed"];

    std::shared_ptr<PathPlannerPath> path = std::make_shared<PathPlannerPath>
    (
        PathPlannerPath
        (
            waypoints,
            rotationTargets,
            pointTowardsZones,
            constraintZones,
            eventMarkers,
            globalConstraints,
            idealStartingState,
            goalEndState,
            reversed
        )
    );

    return path;
}

std::shared_ptr<PPHolonomicDriveController> create_controller()
{
    pathplanner::PIDConstants translationConstants(5.0, 0.0, 0.0);
    pathplanner::PIDConstants rotationConstants(5.0, 0.0, 0.0);
    units::time::second_t period = units::time::millisecond_t(20);

    std::shared_ptr<PPHolonomicDriveController> controller = std::make_shared<PPHolonomicDriveController>
    (
        PPHolonomicDriveController
        (
            translationConstants,
            rotationConstants,
            period
        )
    );

    return controller;
}

class Robot : public frc::TimedRobot
{

public:
    Robot() : frc::TimedRobot()
    {
    }

    bool get_should_flip()
    {
        return false;
    }

    void AutonomousInit() override
    {  
        path = create_path_from_json();
        controller = create_controller();

        std::ifstream f(frc::filesystem::GetDeployDirectory() + "/settings.json");
        wpi::json config = wpi::json::parse(f);
        
        this->follow_path_command.reset
        (
            new FollowPathCommand
            (
                path,
                std::bind(&Drivetrain::get_robot_pose, &m_swerve),
                std::bind(&Drivetrain::get_chassis_speeds, &m_swerve),
                std::bind(&Drivetrain::yield_robot_output, &m_swerve, std::placeholders::_1, std::placeholders::_2),
                controller,
                pathplanner::ExtendedRobotConfig::from_json(config),
                std::bind(&Robot::get_should_flip, this),
                {}
            )
        );

        this->follow_path_command->Initialize();
        
    }

    void AutonomousPeriodic() override
    {
        this->follow_path_command->Execute();
        m_swerve.Drive_Commanded();
    }

    void TeleopPeriodic() override { }

private:
    Drivetrain m_swerve;

    std::shared_ptr<PathPlannerPath> path;
    std::shared_ptr<PPHolonomicDriveController> controller;
    std::shared_ptr<FollowPathCommand> follow_path_command;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
