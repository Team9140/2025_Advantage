package org.team9140.lib.rollers;

import static edu.wpi.first.units.Units.Seconds;

import org.team9140.frc2025.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// Heavily inspired by Team Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/rollers/
public class RollerIOSim implements RollerIO {
    private final DCMotor motor;
    private final DCMotorSim motorSim;
    private byte sign = 1;

    public RollerIOSim(DCMotor motor, double moi, double gear_ratio) {
        this.motor = motor;
        this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, gear_ratio), motor);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            runVolts(0.0);
        }

        motorSim.update(Constants.LOOP_PERIOD.in(Seconds));

        inputs.data = new RollerIOData(
                motorSim.getInputVoltage(),
                motorSim.getCurrentDrawAmps(),
                motor.getCurrent(motorSim.getTorqueNewtonMeters()),
                0.0,
                false,
                true);
    }

    @Override
    public void runVolts(double volts) {
        motorSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0) * sign);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setInverted(boolean inverted) {
        sign = (byte) (inverted ? -1 : 1);
    }
}
