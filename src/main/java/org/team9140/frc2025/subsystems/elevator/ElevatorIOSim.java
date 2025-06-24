package org.team9140.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import org.team9140.frc2025.Constants;

// Heavily inspired by Team Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/superstructure/elevator
public class ElevatorIOSim implements ElevatorIO {
    private static final DCMotor gearbox =
            DCMotor.getKrakenX60Foc(2).withReduction(Constants.Elevator.GEAR_RATIO);

    // My understanding of this is that it is a matrix representing the ordinary
    // output of the elevator without any inputs. Specifically, it will be
    // multiplied (dot product) by a 2x1 matrix called x. The first element of this
    // matrix is the position of the elevator, and the second element is the
    // velocity. This will produce a 2x1 matrix representing the output of the
    // elevator with its first row being velocity and second being acceleration.
    // Note that this resultant matrix will be summed with other 2x1 matrices
    // representing the same things but produced from the results of the inputs to
    // the system. We will call this sum X (this is normally represented by an x
    // with a dot over it) which is the change in position and change in velocity of
    // the elevator with respect to time. We can then integrate X wrt time to get x,
    // the sim-state.
    //
    // Since position doesn't affect the resultant matrix, the first column of
    // matrix A is 0 causing the position of the elevator to be ignored in the
    // calculation of the resulting velocity and acceleration because current
    // position has no affect on how it will change. The top right cell of matrix A
    // is 1 showing that calculated velocity is the same as what is provided.
    // (Recall that matrix A does not factor inputs into the system quite yet.) The
    // bottom right cell of the matrix is used to determine how the back-EMF of a
    // motor affects the acceleration of the elevator. I'm not entirely sure on how
    // that formula is derived, but I'm sure there's information online.
    //
    // Sorry if this explanation is scientifically inaccurate, but I tried my
    // best to explain it in a way that I understand.
    private static final Matrix<N2, N2> A =
            MatBuilder.fill(
                    Nat.N2(),
                    Nat.N2(),
                    0,
                    1,
                    0,
                    -gearbox.KtNMPerAmp // Torque
                            // per Amp
                            // of input
                            / (gearbox.rOhms
                                    * Math.pow(Constants.Elevator.SPOOL_RADIUS.in(Meters), 2)
                                    * Constants.Elevator.mass.in(Kilograms)
                                    * gearbox.KvRadPerSecPerVolt));

    // This contributes to the sum X described above by showing the relation of how
    // current affects acceleration. This vector is multiplied by a scalar, u which
    // is the torque-producing current applied to the motor. The first cell of this
    // vector is 0.0 because current affects acceleration, and not velocity.
    private static final Vector<N2> B =
            VecBuilder.fill(
                    0.0,
                    gearbox.KtNMPerAmp
                            / (Constants.Elevator.SPOOL_RADIUS.in(Meters)
                                    * Constants.Elevator.mass.in(Kilograms)));

    // The sim-state represented by the vector x as described earlier.
    private Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
    private double inputTorqueCurrent = 0.0;
    private double appliedVolts = 0.0;

    private final PIDController controller =
            new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    private boolean closedLoop = false;
    private double feedforward = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (closedLoop) {
            // Run control at 1khz
            for (int i = 0; i < Constants.LOOP_PERIOD.in(Seconds) / (1.0 / 1000.0); i++) {
                setInputTorqueCurrent(
                        controller.calculate(
                                        simState.get(0)
                                                / Constants.Elevator.SPOOL_RADIUS.in(Meters))
                                + feedforward);
                update(1.0 / 1000.0);
            }
        } else {
            controller.reset();
            update(Constants.LOOP_PERIOD.in(Seconds));
        }

        inputs.data =
                new ElevatorIOData(
                        true,
                        true,
                        simState.get(0) / Constants.Elevator.SPOOL_RADIUS.in(Meters),
                        simState.get(1) / Constants.Elevator.SPOOL_RADIUS.in(Meters),
                        appliedVolts,
                        Math.copySign(inputTorqueCurrent, appliedVolts),
                        Math.copySign(inputTorqueCurrent, appliedVolts),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0);
    }

    @Override
    public void runOpenLoop(double output) {
        closedLoop = false;
        setInputTorqueCurrent(output);
    }

    @Override
    public void runVolts(double volts) {
        closedLoop = false;
        setInputVoltage(volts);
    }

    @Override
    public void stop() {
        runOpenLoop(0.0);
    }

    /** Run elevator output shaft to positionRad with additional feedforward output */
    @Override
    public void runPosition(double positionRad, double feedforward) {
        closedLoop = true;
        controller.setSetpoint(positionRad);
        this.feedforward = feedforward;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts = gearbox.getVoltage(gearbox.getTorque(torqueCurrent), simState.get(1));
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    private void setInputVoltage(double voltage) {
        setInputTorqueCurrent(
                gearbox.getCurrent(
                        simState.get(1) / Constants.Elevator.SPOOL_RADIUS.in(Meters), voltage));
    }

    private void update(double dt) {
        inputTorqueCurrent =
                MathUtil.clamp(
                        inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                                A.times(x)
                                        .plus(B.times(u))
                                        .plus(
                                                VecBuilder.fill(
                                                        0.0,
                                                        -Constants.G.in(MetersPerSecondPerSecond)
                                                                * Math.sin(
                                                                        Constants.Elevator
                                                                                .ElevatorAngle.in(
                                                                                Radians)))),
                        simState,
                        MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
                        dt);

        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= 0.0) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, 0.0);
        }
        if (simState.get(0) >= Constants.Elevator.MAX_HEIGHT.in(Meters)) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, Constants.Elevator.MAX_HEIGHT.in(Meters));
        }
    }
}
