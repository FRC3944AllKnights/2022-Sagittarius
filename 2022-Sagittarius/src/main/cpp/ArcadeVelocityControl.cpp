#include "ArcadeVelocityControl.h"

ArcadeVelocityControl::ArcadeVelocityControl(){

}

void ArcadeVelocityControl::DriveInit(){
    Back_Right.RestoreFactoryDefaults();
    Back_Right_PID.SetP(kP);
    Back_Right_PID.SetI(kI);
    Back_Right_PID.SetD(kD);
    Back_Right_PID.SetIZone(kIz);
    Back_Right_PID.SetFF(kFF);
    Back_Right_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Back_Left.RestoreFactoryDefaults();
    Back_Left_PID.SetP(kP);
    Back_Left_PID.SetI(kI);
    Back_Left_PID.SetD(kD);
    Back_Left_PID.SetIZone(kIz);
    Back_Left_PID.SetFF(kFF);
    Back_Left_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Front_Left.RestoreFactoryDefaults();
    Front_Left_PID.SetP(kP);
    Front_Left_PID.SetI(kI);
    Front_Left_PID.SetD(kD);
    Front_Left_PID.SetIZone(kIz);
    Front_Left_PID.SetFF(kFF);
    Front_Left_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Front_Right.RestoreFactoryDefaults();
    Front_Right_PID.SetP(kP);
    Front_Right_PID.SetI(kI);
    Front_Right_PID.SetD(kD);
    Front_Right_PID.SetIZone(kIz);
    Front_Right_PID.SetFF(kFF);
    Front_Right_PID.SetOutputRange(kMinOutput, kMaxOutput);
}

void ArcadeVelocityControl::Drive(double Twist, double Y){
    double twist = deadbandremover(Twist);
    double y = deadbandremover(Y);

    double vTwist = twist/wheelCircumference*gearRatio*60*4;
    double vY = y/wheelCircumference*gearRatio*60*4;

    if (vTwist == 0 and vY == 0){
      Back_Right.Set(0);
      Front_Right.Set(0);

      Back_Left.Set(0);
      Front_Left.Set(0);
    }
    else{
      Back_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);
      Front_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);

      Back_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
      Front_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
    }
}

double ArcadeVelocityControl::deadbandremover(double value){
    double x;
    if(value > -0.1 and value < 0.1){
      value = 0.0;
    }

    if(value < 0){
      x = -1;
    }
    else{
      x = 1;
    }

    value = pow(value, 2.0);
    return value*x;
}