package org.team9140.frc2025.subsystems.manipulator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static org.team9140.frc2025.Constants.Manipulator.HOLD_AMPERAGE_GAME_PIECE;
import static org.team9140.frc2025.Constants.Manipulator.HOLD_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_CORAL;
import static org.team9140.frc2025.Constants.Manipulator.INTOOKEN_TIME;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_CORAL;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.team9140.lib.rollers.RollerIO;
import org.team9140.lib.rollers.RollerIOInputsAutoLogged;

public class Manipulator extends SubsystemBase {
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();
  private RollerIO rollerIO;

  enum ManipulatorState {
    IDLE(0.0),
    INTAKE_ALGAE(INTAKE_VOLTAGE_ALGAE),
    INTAKE_CORAL(INTAKE_VOLTAGE_CORAL),
    OUTTAKE_ALGAE(OUTTAKE_VOLTAGE_ALGAE),
    OUTTAKE_CORAL(OUTTAKE_VOLTAGE_CORAL),
    HOLD_ALGAE(HOLD_VOLTAGE_ALGAE),
    UNSTICK_CORAL(-INTAKE_VOLTAGE_CORAL);

    private double voltage;

    ManipulatorState(double voltage) {
      this.voltage = voltage;
    }

    public double getVoltage() {
      return voltage;
    }
  }

  private ManipulatorState state = ManipulatorState.IDLE;

  enum Holdable {
    WATER,
    ALGAE,
    CORAL
  }

  // Will not switch away from coral unless algae is intooken cuz we can't ensure
  // that the coral actually fell.
  private Holdable currentItem = Holdable.WATER;

  public Manipulator(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
    this.rollerIO.setBrakeMode(true);
    this.rollerIO.setInverted(false);

    this.setDefaultCommand(this.stop());
  }

  public void periodic() {
    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Manipulator", rollerIOInputs);
    rollerIO.runVolts(state.getVoltage());
  }

  public Command intakeAlgae() {
    return this.runOnce(
            () -> {
              this.currentItem = Holdable.ALGAE;
              this.state = ManipulatorState.INTAKE_ALGAE;
            })
        .withName("intake algae");
  }

  public Command intakeCoral() {
    return this.runOnce(
            () -> {
              this.currentItem = Holdable.CORAL;
              this.state = ManipulatorState.INTAKE_CORAL;
            })
        .withName("intake coral");
  }

  public Command outtake() {
    return this.runOnce(
            () -> {
              this.state =
                  switch (currentItem) {
                    case ALGAE -> ManipulatorState.OUTTAKE_ALGAE;
                    case CORAL -> ManipulatorState.OUTTAKE_CORAL;
                    default -> ManipulatorState.IDLE;
                  };

              currentItem = currentItem == Holdable.ALGAE ? Holdable.WATER : currentItem;
            })
        .withName("outtake");
  }

  public Command unstickCoral() {
    return this.runOnce(() -> this.state = ManipulatorState.UNSTICK_CORAL)
        .withName("unstick coral");
  }

  public Command stop() {
    return this.runOnce(
            () -> {
              this.state =
                  switch (this.state) {
                    case INTAKE_ALGAE -> ManipulatorState.HOLD_ALGAE;
                    default -> ManipulatorState.IDLE;
                  };
            })
        .withName("idle (hold or stop)");
  }

  public final Trigger hasCoral = new Trigger(() -> this.currentItem.equals(Holdable.CORAL));
  public final Trigger hasAlgae = new Trigger(() -> this.currentItem.equals(Holdable.ALGAE));

  public final Trigger justIntookenGamePooken =
      new Trigger(
              () ->
                  Amps.of(Math.abs(this.rollerIOInputs.data.torqueCurrentAmps()))
                      .gt(HOLD_AMPERAGE_GAME_PIECE))
          .debounce(INTOOKEN_TIME.in(Seconds), Debouncer.DebounceType.kBoth);
}
