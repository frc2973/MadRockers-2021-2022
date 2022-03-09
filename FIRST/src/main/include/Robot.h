// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/VictorSP.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "CustomController.h"
#include "ports.h"
#include "Limelight.h"

using namespace std;
using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:
  CustomController xbox1;
  CANSparkMax left_f;
  CANSparkMax left_b;
  CANSparkMax right_f;
  CANSparkMax right_b;
  CANSparkMax shooter;
  /*CANSparkMax climb1;
  CANSparkMax climb2;
  float c1_ref, c2_ref;
  SparkMaxRelativeEncoder climb1_en = climb1.GetEncoder();
  SparkMaxRelativeEncoder climb2_en = climb2.GetEncoder();*/
  SparkMaxRelativeEncoder shooter_en = shooter.GetEncoder();
  SparkMaxPIDController shooter_pid = shooter.GetPIDController();
  VictorSP low_feed;
  VictorSP top_feed;
  VictorSP lift;
  VictorSP intake;

  Robot() : 
  xbox1(Ports::XBOX1), 
  left_f(Ports::LEFT_FRONT, CANSparkMax::MotorType::kBrushless), 
  left_b(Ports::LEFT_BACK, CANSparkMax::MotorType::kBrushless), 
  right_f(Ports::RIGHT_FRONT, CANSparkMax::MotorType::kBrushless), 
  right_b(Ports::RIGHT_BACK, CANSparkMax::MotorType::kBrushless),
  shooter(Ports::SHOOTER, CANSparkMax::MotorType::kBrushless),
  //climb1(Ports::CLIMB1, CANSparkMax::MotorType::kBrushless),
  //climb2(Ports::CLIMB2, CANSparkMax::MotorType::kBrushless),
  low_feed(Ports::LOW_FEED),
  top_feed(Ports::TOP_FEED),
  lift(Ports::LIFT),
  intake(Ports::INTAKE) {}

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  /*frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;*/
};
