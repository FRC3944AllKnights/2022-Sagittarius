#ifndef ELEVATOR_H
#define ELEVATOR_H
#include <cmath>
#include <rev/CANSparkMax.h>

class Elevator{
    public:
        
        Elevator();
        void ElevatorBalls(bool ButtTwo, bool ButtThree, bool ButtFour);

    private:

        rev::CANSparkMax ElevatorIntake{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax ElevatorOuttake{7, rev::CANSparkMax::MotorType::kBrushless};

};


#endif