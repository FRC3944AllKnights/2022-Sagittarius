#ifndef ARCADEVELOCITYCONTROL_H
#define ARCADEVELOCITYCONTROL_H
#include <cmath>
#include <rev/CANSparkMax.h>
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include <wpi/fs.h>


class ArcadeVelocityControl{
    public:
        ArcadeVelocityControl();
        void Drive(double X, double Y, double Twist);
        void PureVelocityControl(units::meters_per_second_t x, units::radians_per_second_t theta);
        void DriveInit();

    private:
        double deadbandremover(double value);

        //gear ratio
        double gearRatio = 10.75;
        double wheelCircumference = 0.479;
        double vLeft;
        double vRight;
        frc::DifferentialDriveKinematics m_kinematics{0.6_m};

        rev::CANSparkMax Back_Right{3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Back_Left{4, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Front_Right{2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Front_Left{1, rev::CANSparkMax::MotorType::kBrushless};

        //controller gain variables
        double kP = 9e-5;
        double kI = 1e-7;
        double kD = 0;
        double kIz = 0; 
        double kFF = 0.000015;

        //output parameters
        double kMaxOutput = 1.0; 
        double kMinOutput = -1.0;
        const double MaxRPM = 5700;

        //define the PID controller and encoder for the back right motor
        rev::SparkMaxPIDController Back_Right_PID = Back_Right.GetPIDController();
        rev::SparkMaxRelativeEncoder Back_Right_encoder = Back_Right.GetEncoder();

        rev::SparkMaxPIDController Back_Left_PID = Back_Left.GetPIDController();
        rev::SparkMaxRelativeEncoder Back_Left_encoder = Back_Left.GetEncoder();

        rev::SparkMaxPIDController Front_Right_PID = Front_Right.GetPIDController();
        rev::SparkMaxRelativeEncoder Front_Right_encoder = Front_Right.GetEncoder();

        rev::SparkMaxPIDController Front_Left_PID = Front_Left.GetPIDController();
        rev::SparkMaxRelativeEncoder Front_Left_encoder = Front_Left.GetEncoder();

};

#endif