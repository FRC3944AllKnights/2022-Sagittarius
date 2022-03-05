#ifndef ELEVATOR_H
#define ELEVATOR_H
#include <cmath>
#include <rev/CANSparkMax.h>

class Elevator{
    public:
        
        Elevator();
        void ElevatorBalls(bool ButtThree, bool ButtFour);
        void shooterFeed(double speed);
    private:
        double lastSpeed = 0.0;
        bool disabled = true;
        rev::CANSparkMax ElevatorIntake{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax ElevatorOuttake{7, rev::CANSparkMax::MotorType::kBrushless};

};


#endif