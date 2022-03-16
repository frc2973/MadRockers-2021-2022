// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  SmartDashboard::PutNumber("Set Point", 0);
  SmartDashboard::PutNumber("Wheel Left", 0);
  SmartDashboard::PutNumber("Wheel Right", 0);
  
  shooter_pid.SetP(0); //Initialize PID
  shooter_pid.SetI(0);
  shooter_pid.SetD(0);
  shooter_pid.SetFF(0.00018);
  shooter_pid.SetIZone(0);
  shooter_pid.SetOutputRange(-1.0, 1.0);
  stop_shooter();

  shooting = false; //Initialize trackers
  driven = false;
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
  float set_point = 0.47; //Initialize set point

  l_start = left_en.GetPosition(); //Set trackers
  r_start = right_en.GetPosition();
  driven = false;

  start_shooter(set_point); //Initialize wheels
  intake.Set(0);
  low_feed.Set(0);
  top_feed.Set(0);

  timer.Reset(); //Start timer
  timer.Start();
}

void Robot::AutonomousPeriodic() {
  float distance = 30; //Initialize auto values
  float speed = 0.2;
  float set_point = 0.47;
  
  //Start shooter PID
  if(shooter_en.GetVelocity() > 0.95 * set_point * MaxRPM) {
    start_pid();
  }

  if(!timer.HasElapsed(1.25_s) && !driven) { //Start by lowering the intake
    lift.Set(0.3);
  }
  else if(!driven) { //Start the intake wheels and reset the timer
    lift.Set(0);
    intake.Set(1);
    timer.Stop();
    timer.Reset();
    driven = true;
  }

  //Once the timer is reset, go forwards
  if(abs(left_en.GetPosition() - l_start) < distance && driven) {
    left_f.Set(speed);
    left_b.Set(speed);
  }
  else {
    left_f.Set(0);
    left_b.Set(0);
  }
  if(abs(right_en.GetPosition() - r_start) < distance && driven) {
    right_f.Set(-speed * 1.1);
    right_b.Set(-speed * 1.1);
  }
  else {
    right_f.Set(0);
    right_b.Set(0);
  }
  SmartDashboard::PutNumber("Wheel Left", left_en.GetPosition() - l_start);
  SmartDashboard::PutNumber("Wheel Right", right_en.GetPosition() - r_start);
  
  //After driving, shoot the first ball and start the timer
  if(abs(left_en.GetPosition() - l_start) > distance && 
  abs(right_en.GetPosition() - r_start) > distance && 
  timer.Get() == 0_s) {
    line_up(set_point);
    top_feed.Set(-0.8);
    timer.Start();
  }
  //Shoot the second ball
  if(timer.HasElapsed(2_s) && driven) {
    line_up(set_point);
    low_feed.Set(-0.5);
  }
}

void Robot::TeleopInit() {
  stop_shooter();
  shooting = false;
}

void Robot::TeleopPeriodic() {
  float mult = 0.4; //Reduce speed
  left_f.Set(-xbox1.GetLeftY() * mult); //Drivetrain
  left_b.Set(-xbox1.GetLeftY() * mult);
  right_f.Set(xbox1.GetRightY() * mult);
  right_b.Set(xbox1.GetRightY() * mult);

  low_feed.Set(xbox1.GetLeftBumper() * -0.5); //Shooter system
  top_feed.Set(xbox1.GetRightBumper() * -0.8);
  intake.Set(int(xbox1.GetRightTriggerAxis()));
  lift.Set(xbox1.GetXButton() * 0.3 - xbox1.GetBButton() * 0.5);

  float set_point = SmartDashboard::GetNumber("Set Point", 0);
  if(xbox1.GetAButton()) {
    start_shooter(set_point);
    shooting = true;
  }
  if(shooter_en.GetVelocity() > 0.95 * set_point * MaxRPM && shooting) {
    start_pid();
  }
  if(xbox1.GetYButton()) {
    stop_shooter();
    shooting = false;
  }

  /*if(xbox1.GetAButton()) {
    line_up();
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
