// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "subsystems/ShooterSystem.h"

class Robot;

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    //changed MaxAngularRate from 0.75 to 0.6. makes robot a little slower (2/19/26)//
    //sped up to .625 from .5 foor the Swerve Turning Velocity M.Davin (3/11/26)
    units::radians_per_second_t MaxAngularRate = 0.625_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};



    frc2::CommandXboxController joystick{0};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    ShooterSystem& GetShooter();

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> autoChooser;
    frc::SendableChooser<frc2::Command *> autoChooser1;
    frc::SendableChooser<frc2::Command *> autoChooser2;
   frc::SendableChooser<frc2::Command *> autoChooser3;
   frc::SendableChooser<frc2::Command *> autoChooser4;
    frc::SendableChooser<frc2::Command *> autoChooser5;
    ShooterSystem Shoot1;

    


public:
    RobotContainer();


    frc2::Command *GetAutonomousCommand();


private:
    void ConfigureBindings();
};
