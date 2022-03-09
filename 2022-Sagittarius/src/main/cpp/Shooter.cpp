#include "Shooter.h"

Shooter::Shooter()
{
}

double Shooter::GetPower(double ty){
    double x = (ty-25.3)/(-0.16);
    double weGotSpeedWeGotPower = 0.93;
    double Dwayne = (11.33*x + 1723.81)*weGotSpeedWeGotPower;
    return Dwayne;
}

void Shooter::init()
{
    shooterRight.RestoreFactoryDefaults();

    // set PID coefficients
    shooterRightPID.SetP(kP);
    shooterRightPID.SetI(kI);
    shooterRightPID.SetD(kD);
    shooterRightPID.SetIZone(kIz);
    shooterRightPID.SetFF(kFF);
    shooterRightPID.SetOutputRange(kMinOutput, kMaxOutput);

    shooterRight.Follow(shooterLeft, true);

    shooterLeft.RestoreFactoryDefaults();

    // set PID coefficients
    shooterLeftPID.SetP(kP);
    shooterLeftPID.SetI(kI);
    shooterLeftPID.SetD(kD);
    shooterLeftPID.SetIZone(kIz);
    shooterLeftPID.SetFF(kFF);
    shooterLeftPID.SetOutputRange(kMinOutput, kMaxOutput);

    //shooterLeft.Follow(shooterRight, true);
}

void Shooter::shooterFeed(double speed)
{
    if(isShooting){
        ElevatorOuttake.Set(-1.0);
    }
    else{
        if(elevatorSwitch.Get() == false){
            ElevatorOuttake.Set(0);
        }
        else{
            ElevatorOuttake.Set(speed);
        }
    }
}

void Shooter::ElevatorBalls(bool ButtThree, bool ButtFour)
{
    if (ButtFour)
    {
        Shooter::shooterFeed(0.2);
    }
    else if (ButtThree)
    {
        ElevatorIntake.Set(0.4);
        Shooter::shooterFeed(-0.1);
    }
    else{
        Shooter::shooterFeed(0.0);
        ElevatorIntake.Set(0);
    }
}

void Shooter::spinrev(bool revUp, double ty)
{
    if (revUp)
    {
        //shooterRight.Set(1);
        //shooterRightPID.SetReference(velocity1, rev::ControlType::kVelocity);
        velocity1 = Shooter::GetPower(ty);
        shooterLeftPID.SetReference(-velocity1, rev::ControlType::kVelocity);
        bottomMotor = shooterLeftEncoder.GetVelocity();
        if (bottomMotor <= (-velocity1 + 50))
        {
            isShooting = true;
        }
        else
        {
            isShooting = false;
        }
    }
    else
    {
        isShooting = false;
        shooterLeft.Set(0);
    }

    if (++_loops >= 20)
    {
        _loops = 0;
        _sb.append("\tV1:");
        _sb.append(std::to_string(velocity1));
        _sb.append("\tspd1:");
        _sb.append(std::to_string(shooterRightEncoder.GetVelocity()));
        _sb.append("\tspd2:");
        _sb.append(std::to_string(shooterLeftEncoder.GetVelocity()));

        printf("%s\n", _sb.c_str());
        _sb.clear();
    }
}
