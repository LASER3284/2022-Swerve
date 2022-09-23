/*
 * File created by Charlotte Patton
 */

#pragma once

#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>
#include "SwerveModule.h"

namespace drive {
    class Drive {
        public:
            Drive(frc::Joystick* controller_in);
            void set_joystick(bool control);
            void tick(bool fieldRelative);
            void print();
            void update_odometry();

        private:
            // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
            // to 1.
            frc::SlewRateLimiter<units::scalar> xspeedLimiter{3 / 1_s};
            frc::SlewRateLimiter<units::scalar> yspeedLimiter{3 / 1_s};
            frc::SlewRateLimiter<units::scalar> rotLimiter{3 / 1_s};

            frc::Translation2d frontLeftLocation{+0.36195_m, +0.36195_m};
            frc::Translation2d frontRightLocation{+0.36195_m, -0.36195_m};
            frc::Translation2d backLeftLocation{-0.36195_m, +0.36195_m};
            frc::Translation2d backRightLocation{-0.36195_m, -0.36195_m};
        
            frc::Joystick* controller;
            SwerveModule* frontleft;
            SwerveModule* frontright;
            SwerveModule* backleft;
            SwerveModule* backright;
            bool is_joystickControl;


            AHRS gyro{frc::SerialPort::Port::kMXP};

            frc::SwerveDriveKinematics<4> kinematics{
                frontLeftLocation, frontRightLocation, backLeftLocation,
                backRightLocation};

            frc::SwerveDriveOdometry<4> odometry{kinematics, gyro.GetRotation2d()};
    };
}