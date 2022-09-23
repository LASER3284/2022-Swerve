/*
 * TODO: fix literally everyting :)
 */

#include "SwerveModule.h"

using namespace ctre::phoenix;

drive::SwerveModule::SwerveModule(int drivemotor_in, int turnmotor_in, int encoder_in) {
    // Instantiate each pointer object for WPI_TalonFX and CANCoder
    drivemotor = new motorcontrol::can::WPI_TalonFX(drivemotor_in);
    turnmotor = new motorcontrol::can::WPI_TalonFX(turnmotor_in);
    encoder = new sensors::CANCoder(encoder_in);

    // Limit the PID Controller's input range between -180 and 180 to decrease the maximum travel distance for PID.
    turnPIDController.EnableContinuousInput(
        -180,
        180
    );
    turnPIDController.SetTolerance(2.5);    // For later :)
}

frc::SwerveModuleState drive::SwerveModule::GetState() const {
    return {
        units::meters_per_second_t{drivemotor->GetSelectedSensorVelocity() * 2 * wpi::numbers::pi * kWheelRadius / kEncoderResolution}, // May need to multiply this by 10 in future
        units::radian_t{encoder->GetPosition() * (wpi::numbers::pi / 180) / kEncoderResolution}
    };
}

void drive::SwerveModule::SetDesiredState(const frc::SwerveModuleState& refstate) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
        refstate, frc::Rotation2d((units::degree_t)encoder->GetPosition())
    );

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = drivePIDController.Calculate(
        drivemotor->GetSelectedSensorVelocity(),    // NOTE: This may need to be multiplied by 10 later on...
        state.speed.value()
    );

    const auto driveff = driveFeedforward.Calculate(state.speed);    
    
    double setpoint = (double)state.angle.Degrees();
    if(abs((double)state.speed) <= ((double)(SwerveModule::kMaxSpeed * 0.05))) {
        setpoint = lastAngle;
    }

    turnPIDController.SetSetpoint(setpoint);
    const auto turnOutput = turnPIDController.Calculate(encoder->GetPosition());

    // TODO: get rid of these, but for now keep b/c we're only working on one module
    frc::SmartDashboard::PutNumber("state_angle", (double)state.angle.Degrees());
    frc::SmartDashboard::PutNumber("state_speed", (double)state.speed.value());
    frc::SmartDashboard::PutNumber("pid_setpoint", (double)turnPIDController.GetSetpoint());

    if(!turnPIDController.AtSetpoint()) {    
        frc::SmartDashboard::PutNumber("turn_output_percent", turnOutput);  // Get rid of this with the other ones
        turnmotor->Set(motorcontrol::ControlMode::PercentOutput, turnOutput);
    }
    else {
        turnmotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    }


    // Set the motor outputs.
    drivemotor->SetVoltage(units::volt_t{driveOutput} + driveff);

    lastAngle = (double)state.angle.Degrees();
}

double drive::SwerveModule::getTurnEncPos() {
    return encoder->GetPosition();
}

double drive::SwerveModule::getDriveEncPos() {
    return drivemotor->GetSelectedSensorPosition();
}