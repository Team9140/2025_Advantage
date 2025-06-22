package org.team9140.frc2025.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2025.Constants;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// TODO: SysID
public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.Elevator.CRUISE_VELOCITY.in(RadiansPerSecond),
                    Constants.Elevator.ACCELERATION.in(RadiansPerSecondPerSecond)));
    private State goalState = new State(0, 0);
    private State setpoint = new State(0, 0);

    private Supplier<Distance> goal;
    private Trigger atTarget = new Trigger(() -> Util.epsilonEquals(setpoint.position, goalState.position)
            && Util.epsilonEquals(setpoint.velocity, goalState.velocity));

    public Elevator(ElevatorIO io) {
        this.io = io;

        io.setBrakeMode(true);

        // To prevent making new object each time.
        Distance zero = Meters.of(0.0);
        goal = () -> zero;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        State goalState = new State(
                MathUtil.clamp(goal.get().in(Meters), 0.0, Constants.Elevator.MAX_HEIGHT.in(Meters)),
                0.0);
        double previousVelocity = setpoint.velocity;
        setpoint = profile.calculate(Constants.LOOP_PERIOD.in(Seconds), setpoint, goalState);
        if (setpoint.position < 0.0
                || setpoint.position > Constants.Elevator.MAX_HEIGHT.in(Meters)) {
            setpoint = new State(
                    MathUtil.clamp(setpoint.position, 0.0, Constants.Elevator.MAX_HEIGHT.in(Meters)),
                    0.0);
        }

        double accel = (setpoint.velocity - previousVelocity) / Constants.LOOP_PERIOD.in(Seconds);
        io.runPosition(
                setpoint.position / Constants.Elevator.SPOOL_RADIUS.in(Meters),
                Constants.Elevator.kS * Math.signum(setpoint.velocity) // Magnitude irrelevant
                        + Constants.Elevator.kG
                        + Constants.Elevator.kV * setpoint.velocity // Idk if this is necessary
                        + Constants.Elevator.kA * accel);

        // Log state
        Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
        Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
        Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
        Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    }

    public Trigger getAtTarget() {
        return this.atTarget;
    }

    public void setGoal(Distance goal) {
        setGoal(() -> goal);
    }

    public void setGoal(Supplier<Distance> goal) {
        this.goal = goal;
        this.goalState = new State(
                MathUtil.clamp(goal.get().in(Meters), 0.0, Constants.Elevator.MAX_HEIGHT.in(Meters)),
                0.0);
        this.setpoint = profile.calculate(Constants.LOOP_PERIOD.in(Seconds), setpoint, goalState);
    }

    public Distance getPosition() {
        return Constants.Elevator.SPOOL_RADIUS.times(inputs.data.positionRad());
    }
}
