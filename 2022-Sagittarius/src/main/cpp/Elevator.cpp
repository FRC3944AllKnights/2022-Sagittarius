#include "Elevator.h"

Elevator::Elevator(){}



void Elevator::ElevatorBalls(bool ButtTwo, bool ButtThree, bool ButtFour){

    if (ButtThree){
        ElevatorIntake.Set(0.3);
    }
    else if (ButtFour){
        ElevatorIntake.Set(-0.3);
    }
    else{
        ElevatorIntake.Set(0);
    }
    
    if (ButtTwo){
        ElevatorOuttake.Set(1.0);
    }
    else{
        ElevatorOuttake.Set(0);
    }
}


