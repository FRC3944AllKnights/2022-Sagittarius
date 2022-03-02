#ifndef INTAKE_H
#define INTAKE_H
#include <cmath>
#include <rev/CANSparkMax.h>

class Intake{
    public:
        
        Intake();
        void IntakeBalls(bool Intake, bool Eject);

    private:

        rev::CANSparkMax IntakeMotor{5, rev::CANSparkMax::MotorType::kBrushless};

};


#endif