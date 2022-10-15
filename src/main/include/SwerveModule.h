#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <frc/smartdashboard/SmartDashboard.h>


namespace drive {
    class SwerveModule {
        public:
            SwerveModule(int drivemotor_in, int turnmotor_in, int encoder_in);
            frc::SwerveModuleState GetState() const;
            void SetDesiredState(const frc::SwerveModuleState& state);
            double getTurnEncPos();
            double getDriveEncPos();

            static constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;  // 2 meters per second
            static constexpr units::radians_per_second_t kMaxAngularSpeed{
                wpi::numbers::pi
            };  // 1/2 rotation per second
            
        private:
            static constexpr double kWheelRadius = 0.0508;
            static constexpr int kEncoderResolution = 2048;

            static constexpr auto kModuleMaxAngularVelocity =
                wpi::numbers::pi * 1_rad_per_s;  // pi radians per second
            static constexpr auto kModuleMaxAngularAcceleration =
                wpi::numbers::pi * 2_rad_per_s / 1_s;  // 2pi radians per second^2

            ctre::phoenix::motorcontrol::can::WPI_TalonFX*  drivemotor;
            ctre::phoenix::motorcontrol::can::WPI_TalonFX*  turnmotor;
            ctre::phoenix::sensors::CANCoder*               encoder;

            frc2::PIDController drivePIDController { 0.0001, 0.0, 0.0};
            frc2::PIDController turnPIDController {
                0.011, // P: 0.011
                0.000, // I: 0.00
                0.0000000025, // D: 0.0000000025
            };

            frc::SimpleMotorFeedforward<units::meters> driveFeedforward{ 1_V, 3_V / 1_mps };
            frc::SimpleMotorFeedforward<units::radians> turnFeedforward{ 1_V, 0.5_V / 1_rad_per_s };

            double lastAngle = 0;
    };
}