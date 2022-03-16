//(sus amougs (sus) fortut dub)
#include "Autonomous.h"

Autonomous::Autonomous(){}

void Autonomous::init(bool Redsus){
    
    //init encoders
    encoder1.SetDistancePerPulse(ticks2meters);
    encoder1.SetReverseDirection(true); //reverse one side of the encoders to make them count the same
    encoder2.SetDistancePerPulse(ticks2meters);

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
        _sb.append(std::to_string(pose.Rotation().Radians().to<double>()));
        _sb.append(" vLeft ");
        _sb.append(std::to_string(vLeft));
        _sb.append(" vRight ");
        _sb.append(std::to_string(vRight));
		printf("%s\n",_sb.c_str());
        _sb.clear();
	}    
}

bool Autonomous::FollowTrajectory(bool isRed){
    getOdom();
    if (m_timer.Get() < blueTrajectory.TotalTime()) {
      // Get the desired pose from the trajectory.
        auto desiredPose = redTrajectory.Sample(m_timer.Get());

      // Get the reference chassis speeds from the Ramsete Controller.
      auto refChassisSpeeds = italy.Calculate(pose, desiredPose);

      // Set the linear and angular speeds.
      ramseteOutputX = desiredPose.pose.Translation().X().to<double>();
      ramseteOutputTheta = refChassisSpeeds.omega.to<double>();
      Drive.PureVelocityControl(refChassisSpeeds.vx, refChassisSpeeds.omega);
      return false;
    } else {
      Drive.PureVelocityControl(0_mps, 0_rad_per_s);
      return true;
    }
}

bool Autonomous::TurnRight(double angle){
  getOdom();
  double initialPose = pose.Rotation().Radians().to<double>();
  while(pose.Rotation().Radians().to<double>() - initialPose < angle){
    Drive.PureVelocityControl(0_mps, -3_rad_per_s);
    //Drive.Drive(0.0, 0.3, 0.0);
    getOdom();
  }
  Drive.PureVelocityControl(0_mps, 0_rad_per_s);
  return true;
}



