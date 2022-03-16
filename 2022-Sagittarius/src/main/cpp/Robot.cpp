// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include "ArcadeVelocityControl.h"
#include "Elevator.h"
#include "Intake.h"
#include "Shooter.h"
#include "Pneumatics.h"
#include "turret.h"
#include "Autonomous.h"
//backleft wheel - 4
//frontleft wheel - 1
//backright wheel - 3
//frontright wheel - 2
//intake - 5
//elevator intake - 8
//elevator outtake - 7
//shooter left - 21
//shooter right - 20

bool didpath, didturn, didshoot = false;
int autoSequence = 1;

frc::Joystick joystick{0};
frc::Joystick joystick2{1};
//ArcadeVelocityControl arcadeVelocity;
Intake BallIntake;
Shooter Shoot;
Pneumatics Pneu;
Turret turret;
Autonomous autonomous;

void Robot::RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    autonomous.Drive.DriveInit();
    Shoot.init();
    Pneu.init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
    autoSequence = 1;
    autonomous.init(true);
  }
}

void reset(){
    turret.smartMan(false, false, false, false, 0, 0, 0);
    autonomous.Drive.PureVelocityControl(0_mps, 0_rad_per_s);
    BallIntake.IntakeBalls(false, false);
    Shoot.ElevatorBalls(false, false, false);
    Shoot.spinrev(false, 0);
    autonomous.m_timer.Reset();
}

void blue2balls(){
    switch(autoSequence){
      case 1:{
        turret.smartMan(false, true, false, false, 0, 0, 0);
        Pneu.moveIntake(true, false);
        BallIntake.IntakeBalls(true, false);
        Shoot.ElevatorBalls(true, false, false);
        didpath = autonomous.FollowTrajectory(autonomous.blue1);
        if(didpath){
          didpath = autonomous.TurnRight(1.3);
          reset();
          Pneu.moveIntake(false, true);
          autoSequence = 2;
        }
        break;
      }
      case 2:{
        double Xoffset = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
        double Yoffset = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
        Shoot.ElevatorBalls(true, false, false);
        turret.smartMan(false, false, true, false, Xoffset, Yoffset, 0);
        didshoot = Shoot.spinrev(true, Yoffset);
        if(didshoot){
          reset();
          didturn = autonomous.TurnLeft(1.3);
          autoSequence = 3;
        }
        break;
      }
      case 3:{
        turret.smartMan(false, true, false, false, 0, 0, 0);
        Shoot.ElevatorBalls(true, false, false);
        Pneu.moveIntake(true, false);
        BallIntake.IntakeBalls(true, false);
        didpath = autonomous.FollowTrajectory(autonomous.blue2);
        if(didpath){
          reset();
          autoSequence = 4;
        }
        break;
      }
      default:{
        reset();
        Pneu.moveIntake(false, true);
        break;
      }
    }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    blue2balls();
  }
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
    double Xoffset = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    double Yoffset = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
    double targetArea = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
    double targetSkew = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);

    autonomous.Drive.Drive(joystick.GetX(), joystick.GetY(), joystick.GetTwist());
    Shoot.ElevatorBalls(joystick.GetRawButton(11), joystick.GetRawButton(6), joystick.GetRawButton(9));
    BallIntake.IntakeBalls(joystick.GetRawButton(11), joystick.GetRawButton(12));
    Pneu.moveIntake(joystick.GetRawButton(11), joystick.GetRawButton(12));

    Shoot.spinrev(joystick.GetRawButton(1), Yoffset);
    turret.smartMan(joystick.GetRawButton(7), joystick.GetRawButton(8), joystick.GetRawButton(2), joystick.GetRawButton(1), Xoffset, Yoffset, targetSkew);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
