#include "Elevator.h"

Elevator::Elevator(){}

void Elevator::shooterFeed(double speed){
    if (speed != lastSpeed){
        ElevatorOuttake.Set(speed);
        lastSpeed = speed;
    }
}

void Elevator::ElevatorBalls(bool ButtThree, bool ButtFour){

    if (ButtFour){
        Elevator::shooterFeed(0.2);
        disabled = false;
    }
    else if (ButtThree){
        ElevatorIntake.Set(0.4);
        Elevator::shooterFeed(-0.1);
        disabled = false;
    }
    else if (disabled == false){
        Elevator::shooterFeed(0.0);
        ElevatorIntake.Set(0);
        disabled = true;
    }
}


