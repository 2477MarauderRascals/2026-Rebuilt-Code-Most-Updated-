#pragma once

#include "ctre/phoenix6/HootAutoReplay.hpp"

#include "frc/Timer.h"

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <optional>

#include <frc/XboxController.h>

#include <rev/SparkFlex.h>


   class ShooterSystem : public frc2::SubsystemBase {
 private:
 frc::Timer m_timer;
 public:
  ShooterSystem();
  rev::spark::SparkFlex m_leftMS{4, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_rightMS{5, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_BottomLeftMS{6, rev::spark::SparkFlex::MotorType::kBrushless};
 rev::spark::SparkFlex m_Conveyor{9, rev::spark::SparkFlex::MotorType::kBrushless};
//   void Periodic() override;
//   void SimulationPeriodic() override;
  void ShooterSpeed1();
  void ShooterSpeed2();
  void AutoShoot();
  void ShooterIn();
  void ShooterOut();
  void ShooterStop1();
   void ShooterStop2();
};
