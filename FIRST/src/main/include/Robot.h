// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>

#include "CustomController.h"
#include "ports.h"

using namespace std;
using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:
  CustomController xbox;
  CANSparkMax left_f;
  CANSparkMax left_b;
  CANSparkMax right_f;
  CANSparkMax right_b;

  Robot() : 
  xbox(Ports::XBOX_DRIVER), 
  left_f(Ports::LEFT_FRONT, CANSparkMax::MotorType::kBrushless), 
  left_b(Ports::LEFT_BACK, CANSparkMax::MotorType::kBrushless), 
  right_f(Ports::RIGHT_FRONT, CANSparkMax::MotorType::kBrushless), 
  right_b(Ports::RIGHT_BACK, CANSparkMax::MotorType::kBrushless) {}

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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
