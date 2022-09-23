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

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turnPIDController.EnableContinuousInput(
        -units::radian_t{ wpi::numbers::pi },
        units::radian_t{ wpi::numbers::pi }
    );
    //turnPIDController.SetTolerance(units::radian_t { 5.0 * wpi::numbers::pi / 180 });    // For later :)
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
    
    // TODO: get rid of these, but for now keep b/c we're only working on one module
    frc::SmartDashboard::PutNumber("state_angle", (double)state.angle.Degrees());
    frc::SmartDashboard::PutNumber("state_speed", (double)state.speed.value());

    if(state.angle.Degrees() > 60_deg) {
        auto angle = refstate.angle.Degrees();      // Not sure why this is here? Or why it's using refstate instead of state?
    
        const auto turnOutput = turnPIDController.Calculate(
            units::radian_t{encoder->GetPosition() * (wpi::numbers::pi / 180) / kEncoderResolution},
            state.angle.Radians()
        );

        frc::SmartDashboard::PutNumber("turn_output", (double)turnOutput);  // Get rid of this with the other ones

        turnmotor->SetVoltage(units::volt_t { turnOutput });
    }
    else {
        turnmotor->SetVoltage(0_V);
    }


    // Set the motor outputs.
    drivemotor->SetVoltage(units::volt_t{driveOutput} + driveff);

}

double drive::SwerveModule::getTurnEncPos() {
    return encoder->GetPosition();
}

double drive::SwerveModule::getDriveEncPos() {
    return drivemotor->GetSelectedSensorPosition();
}