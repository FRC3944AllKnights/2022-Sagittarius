#include "Pneumatics.h"

Pneumatics::Pneumatics(){
}

void Pneumatics::init(){
    pcmCompressor.EnableDigital();
}