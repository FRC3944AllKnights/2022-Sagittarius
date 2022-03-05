#ifndef PNEUMATICS_H
#define PNEUMATICS_H
#include <iostream>
#include <string>

#include "frc/Compressor.h"
#include "frc/DoubleSolenoid.h"

class Pneumatics{
    public:
        Pneumatics();
        void init();
        void moveIntake(bool deploy, bool retract);
    private:
        frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
        frc::DoubleSolenoid IntakeSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
        bool switchedDown = false;
};
#endif