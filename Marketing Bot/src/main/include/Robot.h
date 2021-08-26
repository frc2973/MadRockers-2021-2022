// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/VictorSP.h>
#include <rev/CANSparkMax.h>

#include "CustomController.h"
#include "Limelight.h"
#include "Ports.h"

using namespace frc;
using namespace rev;
using namespace std;

class Robot : public frc::TimedRobot {
 public:
  CustomController xbox;
  MecanumDrive driveTrain;
  VictorSP left_front;
  VictorSP left_back;
  VictorSP right_front;
  VictorSP right_back;

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

  Robot() : 
  xbox(Ports::XBOX), 
  driveTrain(left_front, left_back, right_front, right_back), 
  left_front(Ports::LEFT_FRONT), 
  left_back(Ports::LEFT_BACK), 
  right_front(Ports::RIGHT_FRONT), 
  right_back(Ports::RIGHT_BACK) {}

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
