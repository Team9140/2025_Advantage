package org.team9140.frc2025.subsystems.superstructure.manipulator;

import static org.team9140.frc2025.Constants.Manipulator.HOLD_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_CORAL;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_CORAL;

import java.util.Map;

import org.team9140.lib.rollers.RollerIO;
import org.team9140.lib.rollers.RollerIOInputsAutoLogged;

public class Manipulator {
    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();
    private RollerIO rollerIO;

    enum ManipulatorState {
        IDLE,
        INTAKE_ALGAE,
        INTAKE_CORAL,
        OUTTAKE_ALGAE,
        OUTTAKE_CORAL,
        HOLD_ALGAE
    }

    private ManipulatorState state = ManipulatorState.IDLE;
    private Map<ManipulatorState, Double> stateVoltages = Map.of(
        ManipulatorState.IDLE, 0.0,
        ManipulatorState.INTAKE_ALGAE, INTAKE_VOLTAGE_ALGAE,
        ManipulatorState.INTAKE_CORAL, INTAKE_VOLTAGE_CORAL,
        ManipulatorState.OUTTAKE_ALGAE, OUTTAKE_VOLTAGE_ALGAE,
        ManipulatorState.OUTTAKE_CORAL, OUTTAKE_VOLTAGE_CORAL,
        ManipulatorState.HOLD_ALGAE, HOLD_VOLTAGE_ALGAE
    );

    public Manipulator(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        this.rollerIO.setBrakeMode(true);
        this.rollerIO.setInverted(false);
    }

    public void periodic() {
        rollerIO.updateInputs(rollerIOInputs);
        rollerIO.runVolts(stateVoltages.get(state));
    }

    public void setState(ManipulatorState state) {
        this.state = state;
    }
}
