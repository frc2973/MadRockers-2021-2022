// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  SmartDashboard::PutNumber("Set Point", 0);

  //Defaults
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;
  SmartDashboard::PutNumber("P Gain", kP);
  SmartDashboard::PutNumber("I Gain", kI);
  SmartDashboard::PutNumber("D Gain", kD);
  SmartDashboard::PutNumber("I Zone", kIz);
  SmartDashboard::PutNumber("Feed Forward", kFF);
  SmartDashboard::PutNumber("Max Output", kMaxOutput);
  SmartDashboard::PutNumber("Min Output", kMinOutput);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  SmartDashboard::PutNumber("Velocity", shooter_en.GetVelocity());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  kP = SmartDashboard::GetNumber("P Gain", 0);
  kI = SmartDashboard::GetNumber("I Gain", 0);
  kD = SmartDashboard::GetNumber("D Gain", 0);
  kIz = SmartDashboard::GetNumber("I Zone", 0);
  kFF = SmartDashboard::GetNumber("Feed Forward", 0);
  kMaxOutput = SmartDashboard::GetNumber("Max Output", 0);
  kMinOutput = SmartDashboard::GetNumber("Min Output", 0);
  shooter_pid.SetP(kP);
  shooter_pid.SetI(kI);
  shooter_pid.SetD(kD);
  shooter_pid.SetIZone(kIz);
  shooter_pid.SetFF(kFF);
  shooter_pid.SetOutputRange(kMinOutput, kMaxOutput);
}

void Robot::TeleopPeriodic() {
  float mult = 0.4;
  /*left_f.Set(-xbox1.GetLeftY() * mult);
  left_b.Set(-xbox1.GetLeftY() * mult);
  right_f.Set(xbox1.GetRightY() * mult);
  right_b.Set(xbox1.GetRightY() * mult);*/

  low_feed.Set(xbox1.GetLeftBumper() * -0.25);
  top_feed.Set(xbox1.GetRightBumper() * -1);
  lift.Set(xbox1.GetLeftY());

  int MaxRPM = 5700;
  float set_point = SmartDashboard::GetNumber("Set Point", 0);
  shooter_pid.SetReference(set_point * MaxRPM, ControlType::kVelocity);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
