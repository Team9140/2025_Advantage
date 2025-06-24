package org.team9140.lib.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static org.team9140.frc2025.Constants.Manipulator.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RollerIOTalonSRX implements RollerIO {
    TalonSRX motor;
    double output;

    public RollerIOTalonSRX(int motorId) {
        this.motor = new TalonSRX(motorId);

        // TODO: Don't reference Constants in here, take it in as a parameter
        this.motor.configPeakCurrentLimit((int) MANIPULATOR_PEAK_CURRENT_LIMIT.in(Amps));
        this.motor.configPeakCurrentDuration(
                (int) MANIPULATOR_PEAK_CURRENT_DURATION.in(Milliseconds));
        this.motor.configContinuousCurrentLimit(
                (int) MANIPULATOR_CONTINUOUS_CURRENT_LIMIT.in(Amps));
        this.motor.enableCurrentLimit(true);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.data =
                new RollerIOData(
                        this.motor.getMotorOutputVoltage(),
                        this.motor.getSupplyCurrent(),
                        this.motor.getStatorCurrent() * Math.signum(output),
                        0,
                        false,
                        false);
    }

    @Override
    public void runVolts(double volts) {
        this.motor.set(TalonSRXControlMode.PercentOutput, (this.output = volts) / 12.0);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        this.motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.motor.setInverted(inverted);
    }
}
