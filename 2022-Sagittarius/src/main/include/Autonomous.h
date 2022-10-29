#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include "frc/Encoder.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "AHRS.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/DriverStation.h"
#include "frc/I2C.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/controller/RamseteController.h>
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>

#include "ArcadeVelocityControl.h"

class Autonomous{
    public:
        Autonomous();
        void init(frc::Trajectory trajectory);
        void getOdom();
        void GenerateTrajectory();
        bool TurnLeft(double angle);
        bool TurnRight(double angle);
        bool FollowTrajectory(frc::Trajectory trajectory);
        void FollowBounceTrajectory();

        frc::Trajectory blue2ball = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/2ballblue.wpilib.json");
        frc::Trajectory blue2balltofeeder = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/2to4blue.wpilib.json");
        frc::Timer m_timer;

        //declaring it here allows the main robot cpp to use it without throwing sparkmax CAN errors
        ArcadeVelocityControl Drive;

    private:
        int pin1, pin2, name;
        std::string _sb;
        int _loops = 0;
        double ticks2meters = 1/360.0*.4775;
        frc::Encoder encoder1{2,3}; //initialize encoders attached to the RIO (possibly needs inverting)
        frc::Encoder encoder2{0,1};
        AHRS ahrs{frc::I2C::Port::kMXP}; //initialize NavX on the RIO
        frc::Rotation2d gyroAngle{units::degree_t(ahrs.GetAngle())};
        frc::Pose2d pose;
        frc::DifferentialDriveOdometry odom{gyroAngle};

        const double MaxRPM = 5700;
        double gearRatio = 10.75;
        double vLeft;
        double vRight;
        frc::DifferentialDriveKinematics m_kinematics{0.6_m};

        //ramsete
        frc::RamseteController italy{2.0, 0.7};

        //print variables
        double ramseteOutputX;
        double ramseteOutputTheta;
};

#endif