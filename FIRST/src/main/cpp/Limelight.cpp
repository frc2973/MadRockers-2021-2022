#include <string>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "Robot.h"

/* Reference:

Config Panel:
http://limelight.local:5801/
http://10.29.73.11:5801

Camera Stream:
http://10.29.73.11:5800

*/

/**
 * Retrieve a value from the limelight table.
 * 
 * @param variable The variable to retrieve.
 * @param default_value An optional parameter for specifying the defualt value.
 * @return The value of the variable.
 */
double Robot::limelight_get(std::string variable, double default_value = 0.0) {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter")->GetNumber(variable, default_value);
}

/**
 * Set a value in the limelight table.
 * 
 * @param variable The variable to set.
 * @param value The value to set the variable to.
 */
void Robot::limelight_set(std::string variable, double value) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter")->PutNumber(variable, value);
}

void Robot::line_up(float set_point) {
  float range = 0.5;
  float speed = 0.1;
  limelight_set("ledMode", 3); //LED on
  while(shooter_en.GetVelocity() < 0.95 * set_point * MaxRPM) {}
  start_pid();
  if(limelight_get("tv")) { //has target
    while(abs(limelight_get("tx")) > range) {
      if(limelight_get("tx") > 0) {
        left_f.Set(speed);
        left_b.Set(speed);
        right_f.Set(speed);
        right_b.Set(speed);
      }
      else {
        left_f.Set(-speed);
        left_b.Set(-speed);
        right_f.Set(-speed);
        right_b.Set(-speed);
      }
    }
    left_f.Set(0);
    left_b.Set(0);
    right_f.Set(0);
    right_b.Set(0);
  }
  limelight_set("ledMode", 1); //LED off
}
