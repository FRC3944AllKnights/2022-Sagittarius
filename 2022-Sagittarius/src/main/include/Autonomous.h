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
        void init(bool isRed);
        void getOdom();
        void GenerateTrajectory();
        void DriveAndTurn();
        bool FollowTrajectory(bool isRed);
        void FollowBounceTrajectory();

        //declaring it here allows the main robot cpp to use it without throwing sparkmax CAN errors
        ArcadeVelocityControl Drive;

    private:
        int pin1, pin2, name;
        std::string _sb;
        int _loops = 0;
        double ticks2meters = 1/360.0*.4775;
        frc::Encoder encoder1{0,1}; //initialize encoders attached to the RIO (possibly needs inverting)
        frc::Encoder encoder2{2,3};
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
        frc::Timer m_timer;

        //trajectory
        frc::Trajectory redTrajectory = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/test.wpilib.json");
        frc::Trajectory blueTrajectory = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/test.wpilib.json");
        
        bool bounce1go = false;
        bool bounce2go = false;
        bool bounce3go = false;
        bool bounce4go = false;

        frc::Trajectory simpleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(30_in, 30_in, 0_deg),
           {frc::Translation2d(85_in, 60_in), frc::Translation2d(105_in, 85_in), frc::Translation2d(220_in, 90_in), frc::Translation2d(250_in, 85_in),
           frc::Translation2d(260_in, 70_in), frc::Translation2d(275_in, 60_in), frc::Translation2d(300_in, 30_in), frc::Translation2d(330_in, 60_in),
           frc::Translation2d(300_in, 90_in), frc::Translation2d(274_in, 60_in), frc::Translation2d(220_in, 30_in), frc::Translation2d(160_in, 20_in), frc::Translation2d(120_in, 15_in),
           frc::Translation2d(60_in, 90_in)}, frc::Pose2d(30_in, 85_in, 180_deg), frc::TrajectoryConfig(1.4_mps, 1_mps_sq));

        frc::TrajectoryConfig bounceConfig{0.5_mps, 1_mps_sq};
        frc::Trajectory bounce1Trajectory;
        frc::Trajectory bounce2Trajectory;
        frc::Trajectory bounce3Trajectory;
        frc::Trajectory bounce4Trajectory;

        //print variables
        double ramseteOutputX;
        double ramseteOutputTheta;
};

#endif