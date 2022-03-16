#ifndef SHOOTER_H
#define SHOOTER_H
#include <iostream>
#include <string>

#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "frc/Servo.h"

class Shooter{
    public:
        Shooter();
        void init();
        bool spinrev(bool revUp, double ty); // bool up1, bool down1, bool up2, bool down2, bool revUp);
        void ElevatorBalls(bool ButtThree, bool ButtFour);
        void shooterFeed(double speed);
        double GetPower(double ty);

        std::string _sb;
	    int _loops = 0;
        int velocity1 = 3000;
        double bottomMotor = 0;
        double kP = 1.5e-5, kI = 3.3e-7, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;
        const double MaxRPM = 5700;

    private:
        rev::CANSparkMax shooterLeft{11, rev::CANSparkMax::MotorType::kBrushless};
        rev::SparkMaxPIDController shooterLeftPID = shooterLeft.GetPIDController();
        rev::SparkMaxRelativeEncoder shooterLeftEncoder = shooterLeft.GetEncoder();

        rev::CANSparkMax shooterRight{10, rev::CANSparkMax::MotorType::kBrushless};
        rev::SparkMaxPIDController shooterRightPID = shooterRight.GetPIDController();
        rev::SparkMaxRelativeEncoder shooterRightEncoder = shooterRight.GetEncoder();

        double lastSpeed = 0.0;
        bool elevatorClear = true;
        int elevatorClearCount = 0;
        bool isShooting = false;
        rev::CANSparkMax ElevatorIntake{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax ElevatorOuttake{7, rev::CANSparkMax::MotorType::kBrushless};

        frc::DigitalInput elevatorSwitch{9};
};
#endif