// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>
#include <rev/config/SparkFlexConfig.h>
//Updated SHOOTER MOTOR SPEED (2/26/26)//
//Added SparkFlex config (2/19/26)//
Robot::Robot() {/*
   * Create new SPARK MAX configuration objects. These will store the
   * configuration parameters for the SPARK MAXes that we will set below.
   */
  SparkFlexConfig globalConfig;
  SparkFlexConfig LeftMSrConfig;
  SparkFlexConfig RightMSConfig;
  SparkFlexConfig BottomLeftMSConfig;
  SparkFlexConfig IntakeRConfig;
  SparkFlexConfig IntakeUDConfig;
  SparkFlexConfig ConveyorConfig;
  /*
   * Set parameters that will apply to all SPARKs. We will also use this as
   * the left leader config.
   */
  globalConfig.SmartCurrentLimit(80).SetIdleMode(
    //Changed IdleMode: :kBreak to KCoast to match the changes we made on 3/2/26//
      SparkFlexConfig::IdleMode::kCoast).ClosedLoopRampRate(1).closedLoop
    .P(0.0001)
    .I(0)
    .D(0)
    .OutputRange(-1, 1);
  // PID function to make sure kP Rev configuration does not reset once the code is deployed(3/4/26)//
  // Apply the global config and set the leader SPARK for follower mode
  //This should allow the right motor to go at the came time as the left motor//
  //The true next to the m_leftMS makes it inverted (2/24/26)//
 LeftMSrConfig.Apply(globalConfig);
  RightMSConfig.Apply(globalConfig);
  BottomLeftMSConfig.Apply(globalConfig);
  ConveyorConfig.Apply(globalConfig);
  IntakeRConfig.Apply(globalConfig);
 IntakeUDConfig.Apply(globalConfig);
 
  // IntakeUDConfig.softLimit.ForwardSoftLimit(50)
 // .ForwardSoftLimitEnabled(true)
 // .ReverseSoftLimit(-50)
  //.ReverseSoftLimitEnabled(true);


 



  

  /*
   * Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
  ;m_leftMS.Configure(LeftMSrConfig,
                         rev::ResetMode::kResetSafeParameters,
                         rev::PersistMode::kPersistParameters);
  m_rightMS.Configure(RightMSConfig,
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
 m_BottomLeftMS.Configure(BottomLeftMSConfig,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
m_IntakeR.Configure(IntakeRConfig,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
m_IntakeUD.Configure(IntakeUDConfig,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
m_Conveyor.Configure(ConveyorConfig,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);

}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
        auto const driveState = m_container.drivetrain.GetState();
        auto const heading = driveState.Pose.Rotation().Degrees();
        auto const omega = driveState.Speeds.omega;

        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps) {
            m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
    }
        }

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
    }
}  

void Robot::TeleopPeriodic() {/**
   * Get forward and rotation values from the joystick. Invert the joystick's
   * Y value because its forward direction is negative.
   * //motorIdentifier.Set(#) controls the SPEED of the motor. 1= 100% SPEED FORWARD, -1 = 100% SPEED REVERSE, 0 = 0% SPEED OFF
   */
  if (joystick.GetRightTriggerAxis()>=0.8) {
    m_rightMS.Set(0.8);
     m_BottomLeftMS.Set(0.8);
    m_Conveyor.Set(0.1);
    m_leftMS.Set(-0.8);

  std::cout <<"message testing hitting right bumper button " << std::endl;
  } 
  
  else if (joystick.GetRightTriggerAxis()>=0.2){
     m_rightMS.Set(0.5);
     m_BottomLeftMS.Set(0.5);
    m_Conveyor.Set(0.1);
    m_leftMS.Set(-0.5);
  }

  else{
    m_leftMS.Set(0);
    m_rightMS.Set(0);
m_BottomLeftMS.Set(0);
m_Conveyor.Set(0);
  }
//When the button is pressed, set motors to go down to limit and set intake to run and when it hit the limit, 
//stop elevation but still continue running intake. When relased buttoned set motors go to up until you hit the limit//
if (joystick.GetLeftBumper()){
    m_IntakeR.Set(0.5);
    //change GetPosition to 1 if it starts to moves on its own//
    //Doing this should make it so that it doesn't move on its own.// 

//if(m_UDencounder.GetPosition()==1)
//{ m_IntakeUD.Set(0);}else{ m_IntakeUD.Set(0.1);}

  //std::cout <<"message testing hitting left bumper button " << std::endl;
  }else if (joystick.GetRightBumper()){
    m_IntakeR.Set(-0.5);
  }

  else{
    m_IntakeR.Set(0); 
    //change GetPosition to 0 if it starts to move on 

    //if(m_UDencounder.GetPosition()==)
//{ m_IntakeUD.Set(0);}else{ m_IntakeUD.Set(-0.1);}
}
// this should make it so that the intake moves up and down manually (3/4/26)//
//psotive is down negative is up//
if (joystick.GetLeftTriggerAxis()) {
  if (double position_degrees = m_UDencounder.GetPosition()>=0.35){
 m_IntakeUD.Set(-0.08);
sleep(2);
 } 
 
  else if (double position_degrees = m_UDencounder.GetPosition()<=0.05){
 m_IntakeUD.Set(0.05);
sleep(1);
 }

 else {m_IntakeUD.Set(0);}
}
//else if (joystick.GetLeftTriggerAxis()<0.05) {
 //m_IntakeUD.Set(-0.08);
//}

else {
m_IntakeUD.Set(0);
}

}


  /*
   * Apply values to left and right side. We will only need to set the leaders
   * since the other motors are in follower mode.
   */


void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
