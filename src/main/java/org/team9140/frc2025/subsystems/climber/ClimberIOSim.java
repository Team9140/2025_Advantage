package org.team9140.frc2025.subsystems.climber;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team9140.frc2025.Constants;

public class ClimberIOSim implements ClimberIO {
    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2);
    // IDK what the actual value for moi is here, it changes anyway when the bot hangs, so it don't
    // matter, I think?
    private final DCMotorSim sim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(gearbox, 0.1, Constants.Climber.GEAR_RATIO),
                    gearbox);

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            runVolts(0.0);
        }

        sim.update(Constants.LOOP_PERIOD.in(Seconds));
        inputs.data =
                new ClimberIOData(
                        true,
                        true,
                        sim.getAngularPositionRad(),
                        sim.getAngularVelocityRadPerSec(),
                        sim.getInputVoltage(),
                        sim.getCurrentDrawAmps(),
                        gearbox.getCurrent(
                                sim.getAngularVelocityRadPerSec(), sim.getInputVoltage()),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0);
    }

    @Override
    public void runVolts(double volts) {
        sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }
}
