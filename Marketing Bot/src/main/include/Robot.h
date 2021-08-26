// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/RobotDrive.h>
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
  RobotDrive driveTrain;

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

  Robot() : xbox(Ports::XBOX), driveTrain(Ports::LEFT_FRONT, Ports::LEFT_BACK, Ports::RIGHT_FRONT, Ports::RIGHT_BACK) {}

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
