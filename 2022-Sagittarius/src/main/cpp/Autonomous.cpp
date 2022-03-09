//(sus amougs (sus) fortut dub)
#include "Autonomous.h"

Autonomous::Autonomous(){}

void Autonomous::init(bool Redsus){
    
    //init encoders
    encoder1.SetDistancePerPulse(ticks2meters);
    encoder1.SetReverseDirection(true);
    encoder2.SetDistancePerPulse(ticks2meters);

    //init trajectory
    GenerateTrajectory();

     //init odom
    encoder1.Reset();
    encoder2.Reset();
    gyroAngle = {units::degree_t(0)};
    pose = {units::meter_t(0), units::meter_t(0), gyroAngle};
    gyroAngle = {units::degree_t(-ahrs.GetAngle())};
    if (Redsus){
      odom.ResetPosition(redTrajectory.InitialPose(), gyroAngle);
    }else{
        odom.ResetPosition(blueTrajectory.InitialPose(), gyroAngle);
    }
    m_timer.Start();

    m_timer.Reset();
}

void Autonomous::getOdom(){
    /* These functions are compatible w/the WPI Gyro Class */
    frc::SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs.GetAngle());
    frc::SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs.GetRate());
    gyroAngle = {units::degree_t(-ahrs.GetAngle())};
    pose = odom.Update(gyroAngle, units::meter_t(encoder1.GetDistance()), units::meter_t(encoder2.GetDistance()));

    if (++_loops >= 20) {
		_loops = 0;
        _sb.append("\tX: ");
	    _sb.append(std::to_string(pose.Translation().X().to<double>()));
        _sb.append(" Y: ");
        _sb.append(std::to_string(pose.Translation().Y().to<double>()));
        _sb.append(" Theta: ");
        _sb.append(std::to_string(pose.Rotation().Degrees().to<double>()));
        _sb.append(" vLeft ");
        _sb.append(std::to_string(vLeft));
        _sb.append(" vRight ");
        _sb.append(std::to_string(vRight));
		printf("%s\n",_sb.c_str());
        _sb.clear();
	}    
}

void Autonomous::VelocityControl(units::meters_per_second_t x, units::radians_per_second_t theta){
    auto speeds = m_kinematics.ToWheelSpeeds({x, 0_mps, theta});
    vLeft = speeds.left.to<double>()/0.4775*gearRatio*60;
    vRight = -speeds.right.to<double>()/0.4775*gearRatio*60;

    if (vLeft == 0 && vRight == 0){
        LOCDrive.frontLeft.Set(0.0);
        LOCDrive.rearLeft.Set(0.0);
        LOCDrive.frontRight.Set(0.0);
        LOCDrive.rearRight.Set(0.0);
    } else{
        frontLeftPID.SetReference(vLeft, rev::ControlType::kVelocity);
        rearLeftPID.SetReference(vLeft, rev::ControlType::kVelocity);
        frontRightPID.SetReference(vRight, rev::ControlType::kVelocity);
        rearRightPID.SetReference(vRight, rev::ControlType::kVelocity);
    }
}

void Autonomous::GenerateTrajectory(){
    bounceConfig.SetReversed(false);
    bounce1Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
           frc::Pose2d(30_in, 90_in, 0_deg),
           {frc::Translation2d(70_in, 96_in)}, 
           frc::Pose2d(90_in, 150_in, 90_deg), 
           bounceConfig);
}



