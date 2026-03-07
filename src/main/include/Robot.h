// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/HootAutoReplay.hpp"

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <optional>

#include "RobotContainer.h"
#include <frc/XboxController.h>

#include <rev/SparkFlex.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/SparkAbsoluteEncoder.h>
 //Added testing motors (2/19/26)
using namespace rev::spark;

class Robot : public frc::TimedRobot {
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;

private:
  // Initialize the SPARKs
  //Added testing motors (2/19/26)//
  //MS= Motor Shooter (2/19/26)//
  //Intake R is the motor that moves the intake rollers//
  //Inatake UD is the motor that moves the intake up and down and is the one with the sensor//
  //Names can change(//
  SparkFlex m_leftMS{4, SparkFlex::MotorType::kBrushless};
  SparkFlex m_rightMS{5, SparkFlex::MotorType::kBrushless};
  SparkFlex m_BottomLeftMS{6, SparkFlex::MotorType::kBrushless};
  SparkFlex m_IntakeR{7, SparkFlex::MotorType::kBrushless};
  SparkFlex m_IntakeUD{8, SparkFlex::MotorType::kBrushless};
  SparkFlex m_Conveyor{9, SparkFlex::MotorType::kBrushless};
  SparkAbsoluteEncoder m_UDencounder = m_IntakeUD.GetAbsoluteEncoder();
  double EV1=0; 
  double EV2=1; 
      // Joystick # matches where you put the joystick on driver station//
      //ALWAYS PUT AND CHECK THAT IT IS AT JOYSTICK 1 WHEN JUST TESTING REV MOTORS//
  frc::XboxController joystick{1};
  
  static constexpr bool kUseLimelight = false;

    frc2::Command *m_autonomousCommand;

    RobotContainer m_container;

    /* log and replay timestamp and joystick data */
    ctre::phoenix6::HootAutoReplay m_timeAndJoystickReplay = ctre::phoenix6::HootAutoReplay{}
        .WithTimestampReplay()
        .WithJoystickReplay();
};
