// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive_Commanded()
{
  auto dt = units::time::millisecond_t(20_ms);
  auto states =
    m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
        commanded_chassis_speeds,
        dt));
        
  chassis_speeds = commanded_chassis_speeds;

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::yield_robot_output
        (const frc::ChassisSpeeds& speeds, 
        const pathplanner::DriveFeedforwards& feedforwards)
{
  this->commanded_chassis_speeds = speeds;
}

void Drivetrain::UpdateOdometry() {
  robot_pose = m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

frc::Pose2d &Drivetrain::get_robot_pose()
{
  return this->robot_pose;
}

frc::ChassisSpeeds &Drivetrain::get_chassis_speeds()
{
  return this->chassis_speeds;
}
