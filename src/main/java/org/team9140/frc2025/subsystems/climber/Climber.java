package org.team9140.frc2025.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2025.Constants;

public class Climber extends SubsystemBase {
    private final ClimberIO motors;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO climberIO) {
        this.motors = climberIO;
    }

    @Override
    public void periodic() {
        motors.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command prep() {
        return this.runOnce(() -> this.motors.runVolts(Constants.Climber.PREP_VOLTAGE.in(Volts)))
                .andThen(
                        new WaitUntilCommand(
                                () ->
                                        this.inputs.data.positionRad()
                                                > Constants.Climber.PREPPED_POSITION.in(Radians)))
                .andThen(this.off());
    }

    public Command climb(Supplier<Double> leftTrigger, Supplier<Double> rightTrigger) {
        return this.run(
                () -> {
                    this.motors.runVolts(
                            Constants.Climber.MAX_OUTPUT
                                    .times(leftTrigger.get() - rightTrigger.get() / 4.0)
                                    .in(Volts));
                });
    }

    public Command off() {
        return this.runOnce(this.motors::stop);
    }
}
