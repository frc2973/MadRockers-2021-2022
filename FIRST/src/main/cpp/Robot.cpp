// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  /*m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);*/

  SmartDashboard::PutNumber("Set Point", 0);

  double kP = 0.00025, kI = 0.000001, kD = 0, kIz = 0, kFF = 0.00018, kMaxOutput = 1.0, kMinOutput = -1.0;
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
  SmartDashboard::PutNumber("Velocity 1", shooter_en.GetVelocity());
  SmartDashboard::PutNumber("Velocity 2", shooter_en.GetVelocity());
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
  /*m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
}

void Robot::AutonomousPeriodic() {
  /*if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
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

  shooter_pid.SetReference(0, ControlType::kVelocity);
  shooter.Set(0);

  /*c1_ref = climb1_en.GetPosition();
  c2_ref = climb2_en.GetPosition();*/
}

void Robot::TeleopPeriodic() {
  float mult = 0.4;
  left_f.Set(-xbox1.GetLeftY() * mult);
  left_b.Set(-xbox1.GetLeftY() * mult);
  right_f.Set(xbox1.GetRightY() * mult);
  right_b.Set(xbox1.GetRightY() * mult);

  low_feed.Set(xbox1.GetLeftBumper() * -0.5);
  top_feed.Set(xbox1.GetRightBumper() * -0.8);
  intake.Set(xbox1.GetAButton());
  lift.Set(xbox1.GetLeftY());

  int MaxRPM = 5700;
  float set_point = SmartDashboard::GetNumber("Set Point", 0);
  if(xbox1.GetAButton()) {
    shooter_pid.SetReference(set_point * MaxRPM, ControlType::kVelocity);
  }
  if(xbox1.GetBButton()) {
    shooter_pid.SetReference(0, ControlType::kVelocity);
    //while(shooter_en.GetVelocity() > 10) {}
    shooter.Set(0);
  }

  /*float c1_en = climb1_en.GetPosition() - c1_ref;
  float c2_en = climb2_en.GetPosition() - c2_ref;
  SmartDashboard::PutNumber("C1 Pos", c1_en);
  SmartDashboard::PutNumber("C2 Pos", c2_en);
  float speed = xbox1.GetLeftY();
  climb1.Set(speed * mult);
  climb2.Set(speed * mult * 0.85);
  if(abs(speed) > 0.1) {
    if(c1_en - c2_en > 1) {
      while(c1_en - c2_en > 0.1) {
        //climb1.Set(speed * mult * 0.9);
        climb2.Set(speed * mult * 0.85);
      }
    }
    if(c2_en - c1_en > 1) {
      while(c2_en - c1_en > 0.1) {
        climb1.Set(speed * mult);
        //climb2.Set(speed * mult * 0.75);
      }
    }
  }*/

  /*float range = 0.5;
  float speed = 0.1;

  if(xbox1.GetAButton()) {
    limelight_set("ledMode", 3); //LED on
    while(xbox1.GetAButton()) {}
    if(limelight_get("tv")) { //has target
      while(abs(limelight_get("tx")) > range) {
        if(limelight_get("tx") > 0) {
          //left_f.Set(speed);
          //left_b.Set(speed);
          right_f.Set(speed);
          right_b.Set(speed);
        }
        else {
          left_f.Set(-speed);
          left_b.Set(-speed);
          //right_f.Set(-speed);
          //right_b.Set(-speed);
        }
      }
      left_f.Set(0);
      left_b.Set(0);
      right_f.Set(0);
      right_b.Set(0);
    }
    limelight_set("ledMode", 1); //LED off
  }

  if(xbox1.GetBButton()) {
    limelight_set("ledMode", 3);
  }

  if(xbox1.GetXButton()) {
    limelight_set("ledMode", 1);
  }*/
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
