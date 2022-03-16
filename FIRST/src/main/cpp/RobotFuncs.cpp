#include "Robot.h"

void Robot::start_shooter(float set_point) {
  shooter_pid.SetP(0);
  shooter_pid.SetI(0);
  shooter_pid.SetReference(set_point * MaxRPM, ControlType::kVelocity);
}

void Robot::start_pid() {
  shooter_pid.SetP(0.00025);
  shooter_pid.SetI(0.000001);
}

void Robot::stop_shooter() {
  shooter_pid.SetReference(0, ControlType::kVelocity);
  shooter.Set(0);
}
