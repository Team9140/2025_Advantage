package org.team9140.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2025.Constants;
import org.team9140.lib.Util;

// TODO: SysID
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            Constants.Elevator.CRUISE_VELOCITY.in(MetersPerSecond),
                            Constants.Elevator.ACCELERATION.in(MetersPerSecondPerSecond)));
    private State goalState = new State(0, 0);
    private State setpoint = new State(0, 0);

    private Supplier<Distance> goal;
    private Trigger atTarget =
            new Trigger(
                    () ->
                            Util.epsilonEquals(setpoint.position, goalState.position)
                                    && Util.epsilonEquals(setpoint.velocity, goalState.velocity));

    private final ElevatorVisualizer measuredVisualizer = new ElevatorVisualizer("Measured");
    private final ElevatorVisualizer setpointVisualizer = new ElevatorVisualizer("Setpoint");
    private final ElevatorVisualizer goalVisualizer = new ElevatorVisualizer("Goal");

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

        State goalState =
                new State(
                        MathUtil.clamp(
                                goal.get().in(Meters),
                                0.0,
                                Constants.Elevator.MAX_HEIGHT.in(Meters)),
                        0.0);
        double previousVelocity = setpoint.velocity;
        setpoint = profile.calculate(Constants.LOOP_PERIOD.in(Seconds), setpoint, goalState);
        if (setpoint.position < 0.0
                || setpoint.position > Constants.Elevator.MAX_HEIGHT.in(Meters)) {
            setpoint =
                    new State(
                            MathUtil.clamp(
                                    setpoint.position,
                                    0.0,
                                    Constants.Elevator.MAX_HEIGHT.in(Meters)),
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

        measuredVisualizer.update(getPosition().in(Meters));
        setpointVisualizer.update(setpoint.position);
        goalVisualizer.update(goalState.position);
    }

    public Trigger getAtTarget() {
        return this.atTarget;
    }

    public Command setGoal(Distance goal) {
        return setGoal(() -> goal);
    }

    public Command setGoal(Supplier<Distance> goal) {
        return this.runOnce(
                () -> {
                    this.goal = goal;
                    this.goalState =
                            new State(
                                    MathUtil.clamp(
                                            goal.get().in(Meters),
                                            0.0,
                                            Constants.Elevator.MAX_HEIGHT.in(Meters)),
                                    0.0);
                    this.setpoint =
                            profile.calculate(
                                    Constants.LOOP_PERIOD.in(Seconds), setpoint, goalState);
                });
    }

    public Command moveToPosition(Distance position) {
        return setGoal(position).andThen(new WaitUntilCommand(atPosition));
    }

    public Command moveToPosition(Supplier<Distance> position) {
        return setGoal(position).andThen(new WaitUntilCommand(atPosition));
    }

    public Distance getPosition() {
        return Constants.Elevator.SPOOL_RADIUS.times(inputs.data.positionRad());
    }

    public final Trigger isUp = new Trigger(() -> this.getPosition().gt(Feet.of(4)));
    private final Distance algaeingCenter =
            Constants.Elevator.L3_ALGAE_height.plus(Constants.Elevator.L2_ALGAE_height).div(2.0);
    public final Trigger isAlgaeing =
            new Trigger(() -> this.getPosition().isNear(algaeingCenter, Inches.of(18.0)));
    public final Trigger atPosition =
            new Trigger(
                    () ->
                            this.getPosition()
                                    .isNear(this.goal.get(), Constants.Elevator.POSITION_epsilon));
    public final Trigger isStowed =
            new Trigger(
                    () ->
                            this.getPosition()
                                    .isNear(Constants.Elevator.STOW_height, Inches.of(0.75)));
}
