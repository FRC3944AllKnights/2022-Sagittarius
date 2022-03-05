#ifndef SHOOTER_H
#define SHOOTER_H
#include <iostream>
#include <string>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "frc/Servo.h"

class Shooter{
    public:
        Shooter();
        void init();
        void spinrev(bool revUp); // bool up1, bool down1, bool up2, bool down2, bool revUp);
        void ElevatorBalls(bool ButtThree, bool ButtFour);
        void shooterFeed(double speed);

        std::string _sb;
	    int _loops = 0;
        int velocity1 = 4000;
        double bottomMotor = 0;
        double kP = 5e-4, kI = 5e-7, kD = 1e-8, kIz = 0, kFF = 0, kMaxOutput = 1.0, kMinOutput = -1.0;
        const double MaxRPM = 5700;

    private:
        rev::CANSparkMax shooterLeft{11, rev::CANSparkMax::MotorType::kBrushless};
        rev::SparkMaxPIDController shooterLeftPID = shooterLeft.GetPIDController();
        rev::SparkMaxRelativeEncoder shooterLeftEncoder = shooterLeft.GetEncoder();

        rev::CANSparkMax shooterRight{10, rev::CANSparkMax::MotorType::kBrushless};
        rev::SparkMaxPIDController shooterRightPID = shooterRight.GetPIDController();
        rev::SparkMaxRelativeEncoder shooterRightEncoder = shooterRight.GetEncoder();

        double lastSpeed = 0.0;
        bool disabled = true;
        rev::CANSparkMax ElevatorIntake{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax ElevatorOuttake{7, rev::CANSparkMax::MotorType::kBrushless};
};
#endif