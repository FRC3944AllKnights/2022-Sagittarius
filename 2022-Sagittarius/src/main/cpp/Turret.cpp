#include "Turret.h"
#include "math.h"

Turret::Turret(){
}

void Turret::init(){
    turretSpinner.SetSelectedSensorPosition(0);
}

void Turret::safetyController(double speed){
    int encoder = turretSpinner.GetSelectedSensorPosition();
    int limit = 1000;
    if(speed > 0){
        if(encoder <= limit){//turretLimitRight.Get()){
            turretSpinner.Set(ControlMode::PercentOutput, speed);
        }
        else{
            turretSpinner.Set(ControlMode::PercentOutput, 0);
        }
    }
    else{
        if(encoder >= -limit){//turretLimitLeft.Get()){
            turretSpinner.Set(ControlMode::PercentOutput, speed);
        }
        else{
            turretSpinner.Set(ControlMode::PercentOutput, 0);
        }
    }
}

void Turret::smartMan(bool left, bool right, bool aimybot, bool shooting, double xPos, double yPos, double skew){

    d = (h2 - h1)/tan(alpha + (yPos*3.14159265/180));
    
    if (right)
    {
        Turret::safetyController(1);
    }
    else if (left){
        Turret::safetyController(-1); 
    }
    else if (aimybot or shooting){
        //sum = sum - cameraPos*0.0015;
        Turret::safetyController((xPos + 2.0)*0.2 + sum);
    }
    else{
        Turret::safetyController(0);
    }

    /*if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tangle in degrees");
	    _sb.append(std::to_string(yPos));
	    _sb.append("\tdistance in inches");
	    _sb.append(std::to_string(d));

        

		printf("%s\n",_sb.c_str());
        _sb.clear();
	}*/
}
    