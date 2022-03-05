#include "Shooter.h"

Shooter::Shooter(){
}

void Shooter::init(){
 shooterLeft.RestoreFactoryDefaults();
    
    // set PID coefficients
    shooterLeftPID.SetP(kP);
    shooterLeftPID.SetI(kI);
    shooterLeftPID.SetD(kD);
    shooterLeftPID.SetIZone(kIz);
    shooterLeftPID.SetFF(kFF);
    shooterLeftPID.SetOutputRange(kMinOutput, kMaxOutput);

 shooterLeft.RestoreFactoryDefaults();
    
    // set PID coefficients
    shooterLeftPID.SetP(kP);
    shooterLeftPID.SetI(kI);
    shooterLeftPID.SetD(kD);
    shooterLeftPID.SetIZone(kIz);
    shooterLeftPID.SetFF(kFF);
    shooterLeftPID.SetOutputRange(kMinOutput, kMaxOutput);
}

void Shooter::spinrev(bool revUp){ //bool up1, bool down1, bool up2, bool down2, bool revUp){

/*
    if (up1 && up1pressed == false){
        up1pressed = true;
        velocity1 += 50;
    }
    else if (up1 == false && up1pressed){
        up1pressed = false;
    }

    if (down1 && down1pressed == false){
        down1pressed = true;
        velocity1 -= 50;
    }
    else if (down1 == false && down1pressed){
        down1pressed = false;
    }

    if (up2 && up2pressed == false){
        up2pressed = true;
        velocity2 += 50;
    }
    else if (up2 == false && up2pressed){
        up2pressed = false;
    }

    if (down2 && down2pressed == false){
        down2pressed = true;
        velocity2 -= 50;
    }
    else if (down2 == false && down2pressed){
        down2pressed = false;
    }
*/
    if (revUp){
        shooterRight.Set(1);
        shooterLeft.Set(-1);
        //shooterRightPID.SetReference(velocity1, rev::ControlType::kVelocity);
        //shooterLeftPID.SetReference(-velocity1, rev::ControlType::kVelocity);
        //if (bottomMotor >= (velocity1 - 200) && bottomMotor <= (velocity1 + 200)){
        //    feeder.Set(ControlMode::PercentOutput, -1);
        //}
        //else{
        //    feeder.Set(ControlMode::PercentOutput, 0);
        //}
    }
    //else if(holdBall){
    //    feeder.Set(ControlMode::PercentOutput, -0.3);
    //}
    else{
        shooterRight.Set(0);
        shooterLeft.Set(0);
    }

    /* if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tV1:");
	    _sb.append(std::to_string(velocity1));
	    _sb.append("\tspd1:");
	    _sb.append(std::to_string(shooterRightEncoder.GetVelocity()));

        _sb.append("\tV2:");
	    _sb.append(std::to_string(velocity2));
	    _sb.append("\tspd2:");
	    _sb.append(std::to_string(shooterLeftEncoder.GetVelocity()));

		//printf("%s\n",_sb.c_str());
        _sb.clear();
	}*/
}
    