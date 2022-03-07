#include "Pneumatics.h"

Pneumatics::Pneumatics(){
}

void Pneumatics::init(){
    pcmCompressor.EnableDigital();
    IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Pneumatics::moveIntake(bool deploy, bool retract){
    if(deploy){
        if(switchedDown == false){
            IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
            switchedDown = true;
        }
    }
    else{
        if (switchedDown){
            IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
            switchedDown = false;
        }
    }
}