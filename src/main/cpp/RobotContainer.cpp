// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Commands.h>


RobotContainer::RobotContainer()
{

//     std::cout<<"testing"<<pathplanner::AutoBuilder::isConfigured()<<std::endl;

//          pathplanner::NamedCommands::registerCommand("Shooter", std::move(
//         frc2::cmd::Run(
//             [this] {Shoot1.AutoShoot();},
//         {&Shoot1}
//     ).WithTimeout(units::second_t(5))
// ));

    //autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    // autoChooser1 = pathplanner::AutoBuilder::buildAutoChooser("RED LEFT AUTO");
    // autoChooser2 = pathplanner::AutoBuilder::buildAutoChooser("RED RIGHT AUTO");
    // autoChooser3 = pathplanner::AutoBuilder::buildAutoChooser("BLUE LEFT AUTO");
    // autoChooser4 = pathplanner::AutoBuilder::buildAutoChooser("BLUE RIGHT AUTO");
    //autoChooser5 = pathplanner::AutoBuilder::buildAutoChooser("SHOOTING AUTO");
    //frc::SmartDashboard::PutData("Auto Choosper", &autoChooser);
      //frc::SmartDashboard::PutData("Auto Mode", &autoChooser1);
    //  frc::SmartDashboard::PutData("Auto Mode", &autoChooser2);
    //  frc::SmartDashboard::PutData("Auto Mode", &autoChooser3);
    //  frc::SmartDashboard::PutData("Auto Mode", &autoChooser4);
   // frc::SmartDashboard::PutData("Auto Mode", &autoChooser5);


    ConfigureBindings();
}

ShooterSystem& RobotContainer::GetShooter(){
    return Shoot1;
}



void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // Changed the joystick.B, to joystick.X M.Davin 3/11/26
    joystick.X().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    joystick.POVUp().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(0.5_mps).WithVelocityY(0_mps);
        })
    );
    joystick.POVDown().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(-0.5_mps).WithVelocityY(0_mps);
        })
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.B().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    // Using the B button for Mark to calibrate pigeon heading to be set to zero when pressed, M.Davin (3/11/26)

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
