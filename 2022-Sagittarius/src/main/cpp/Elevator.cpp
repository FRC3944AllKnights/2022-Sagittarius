#include "Elevator.h"

Elevator::Elevator(){}



void Elevator::ElevatorBalls(bool ButtTwo, bool ButtThree, bool ButtFour){

    if (ButtFour){
        //ElevatorIntake.Set(0.4);
        ElevatorOuttake.Set(0.2);
    }
    else if (ButtThree){
        ElevatorIntake.Set(0.4);
        ElevatorOuttake.Set(-0.1);
    }
    else if (ButtTwo){
        ElevatorOuttake.Set(-1.0);
    }
    else{
        ElevatorOuttake.Set(0);
        ElevatorIntake.Set(0);
    }
}


