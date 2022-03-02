#ifndef TURRET_H
#define TURRET_H
#include <iostream>
#include <string>
#include <frc/DigitalInput.h>

#include "ctre/Phoenix.h"

class Turret{
    public:
        Turret();
        void init();
        void smartMan(bool left, bool right, bool aimybot, double xPos, double yPos, double skew);
        void safetyController(double speed);
        std::string _sb;
	    int _loops = 0;
        double sum = 0;
        double h1 = 40;      //height of robot in inches
        double h2 = 89.5;    //height of target centerpoint in inches
        double alpha = 14.7*3.14159265/180; //angle of camera on robot in rads
        double d;
    
    private:
        WPI_TalonSRX turretSpinner{21};

        frc::DigitalInput turretLimitLeft{4};
        frc::DigitalInput turretLimitRight{3};
       
};
#endif