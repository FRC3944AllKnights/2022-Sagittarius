#ifndef PNEUMATICS_H
#define PNEUMATICS_H
#include <iostream>
#include <string>

#include "frc/Compressor.h"

class Pneumatics{
    public:
        Pneumatics();
        void init();
        void moveIntake(bool deploy);
    private:
        frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};

};
#endif