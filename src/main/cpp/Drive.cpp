/*
 * TODO: fix literally everyting :)
 */

#include "Drive.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

drive::Drive::Drive(frc::Joystick* controller_in) {
    controller = controller_in;
    frontleft = new SwerveModule(10, 9, 16);
    frontright = new SwerveModule(7, 8, 15);
    backleft = new SwerveModule(11, 12, 2);
    backright = new SwerveModule(14, 13, 3);
    gyro.Reset();
}

void drive::Drive::set_joystick(bool state) {
    is_joystickControl = state;
}

void drive::Drive::tick(bool fieldRelative) {
    // Reset the gyro if the driver presses start.
    // This'll help avoid drift in field-oriented mode.
    if(controller->GetRawButtonReleased(8)) {
        gyro.Reset();
    }

    double yAxis = frc::ApplyDeadband(-controller->GetRawAxis(1), 0.15);
    double xAxis = frc::ApplyDeadband(-controller->GetRawAxis(0), 0.15);
    double rAxis = frc::ApplyDeadband(-controller->GetRawAxis(4), 0.15);

    const auto translation = frc::Translation2d((units::meter_t)yAxis, (units::meter_t)xAxis) * (double)SwerveModule::kMaxSpeed;

    const auto rotation = (rAxis * SwerveModule::kMaxAngularSpeed);

    frc::ChassisSpeeds nonrelspeeds = frc::ChassisSpeeds();
    nonrelspeeds.omega = rotation;
    nonrelspeeds.vx = translation.X() / 1_s;
    nonrelspeeds.vy = translation.Y() / 1_s;

    const auto relspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        translation.X() / 1_s,
        translation.Y() / 1_s,
        rotation, 
        gyro.GetRotation2d()
    );

    auto states = kinematics.ToSwerveModuleStates(fieldRelative ? 
        relspeeds :
        nonrelspeeds
    );

    kinematics.DesaturateWheelSpeeds(&states, SwerveModule::kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frontleft->SetDesiredState(fl);
    frontright->SetDesiredState(fr);
    backleft->SetDesiredState(bl);
    backright->SetDesiredState(br);
}

void drive::Drive::print() {
    frc::SmartDashboard::PutNumber("fl_turn", frontleft->getTurnEncPos());
    frc::SmartDashboard::PutNumber("fl_drive", frontleft->getDriveEncPos());
    frc::SmartDashboard::PutNumber("fr_turn", frontright->getTurnEncPos());
    frc::SmartDashboard::PutNumber("fr_drive", frontright->getDriveEncPos());
    frc::SmartDashboard::PutNumber("bl_turn", backleft->getTurnEncPos());
    frc::SmartDashboard::PutNumber("bl_drive", backleft->getDriveEncPos());
    frc::SmartDashboard::PutNumber("br_turn", backright->getTurnEncPos());
    frc::SmartDashboard::PutNumber("br_drive", backright->getDriveEncPos());
}

void drive::Drive::update_odometry() {
    odometry.Update(gyro.GetRotation2d(), frontleft->GetState(),
        frontright->GetState(), backleft->GetState(),
        backright->GetState()
    );
}