#include "Intake.h"

Intake::Intake(){}



void Intake::IntakeBalls(bool Intake, bool Eject){

    if (Intake){
        IntakeMotor.Set(0.3);
    }
    else if (Eject){
        IntakeMotor.Set(-0.3);
    }
    else{
        IntakeMotor.Set(0);
    }
}


