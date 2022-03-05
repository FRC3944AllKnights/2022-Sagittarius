#include "Shooter.h"

Shooter::Shooter(){
}

void Shooter::init(){
 shooterRight.RestoreFactoryDefaults();
    
    // set PID coefficients
    shooterRightPID.SetP(kP);
    shooterRightPID.SetI(kI);
    shooterRightPID.SetD(kD);
    shooterRightPID.SetIZone(kIz);
    shooterRightPID.SetFF(kFF);
    shooterRightPID.SetOutputRange(kMinOutput, kMaxOutput);

 shooterLeft.RestoreFactoryDefaults();
    
    // set PID coefficients
    shooterLeftPID.SetP(kP);
    shooterLeftPID.SetI(kI);
    shooterLeftPID.SetD(kD);
    shooterLeftPID.SetIZone(kIz);
    shooterLeftPID.SetFF(kFF);
    shooterLeftPID.SetOutputRange(kMinOutput, kMaxOutput);
}

void Shooter::shooterFeed(double speed){
    if (speed != lastSpeed){
        ElevatorOuttake.Set(speed);
        lastSpeed = speed;
    }
}

void Shooter::ElevatorBalls(bool ButtThree, bool ButtFour){
    if (ButtFour){
        Shooter::shooterFeed(0.2);
        disabled = false;
    }
    else if (ButtThree){
        ElevatorIntake.Set(0.4);
        Shooter::shooterFeed(-0.1);
        disabled = false;
    }
    else if (disabled == false){
        Shooter::shooterFeed(0.0);
        ElevatorIntake.Set(0);
        disabled = true;
    }
}

void Shooter::spinrev(bool revUp){
    if (revUp){
        shooterRight.Set(1);
        shooterLeft.Set(-1);
        //shooterRightPID.SetReference(velocity1, rev::ControlType::kVelocity);
        //shooterLeftPID.SetReference(-velocity1, rev::ControlType::kVelocity);
        bottomMotor = shooterRightEncoder.GetVelocity();
        if (bottomMotor >= (velocity1 - 200) && bottomMotor <= (velocity1 + 200)){
            Shooter::shooterFeed(-1.0);
            disabled = false;
        }
        else{
            if(disabled == false){
                Shooter::shooterFeed(0.0);
                disabled = true;
            }
        }
    }
    else{
        shooterRight.Set(0);
        shooterLeft.Set(0);
    }

     if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tV1:");
	    _sb.append(std::to_string(velocity1));
	    _sb.append("\tspd1:");
	    _sb.append(std::to_string(shooterRightEncoder.GetVelocity()));

		//printf("%s\n",_sb.c_str());
        _sb.clear();
	}
}
    