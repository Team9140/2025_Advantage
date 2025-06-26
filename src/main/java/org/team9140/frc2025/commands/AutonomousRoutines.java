package org.team9140.frc2025.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Optional;
import java.util.function.Supplier;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.ElevatorSetbacks;
import org.team9140.frc2025.Robot;
import org.team9140.frc2025.helpers.AutoAiming;
import org.team9140.frc2025.subsystems.drive.Drive;
import org.team9140.frc2025.subsystems.elevator.Elevator;
import org.team9140.frc2025.subsystems.funnel.Funnel;
import org.team9140.frc2025.subsystems.manipulator.Manipulator;
import org.team9140.lib.FollowPath;

public class AutonomousRoutines {
    private static final Time INTAKE_TIME = Seconds.of(1.0);
    private static final Time THROW_TIME = Seconds.of(0.25);
    private final Drive drivetrain;

    // private final Supplier<Command> SCORE_CORAL_L4;
    private final Supplier<Command> INTAKE_CORAL;
    private final Supplier<Command> STOP_INTAKE;
    // private final Supplier<Command> ARM_HALFWAY;
    // private final Function<Boolean, Command> REEF_DRIVE_THEN_SCORE_L4;

    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Funnel funnel;

    public AutonomousRoutines(
            Drive drivetrain, Elevator elevator, Manipulator manipulator, Funnel funnel) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.funnel = funnel;

        // this.SCORE_CORAL_L4 = () ->
        // elevator.moveToPosition(Constants.Elevator.L4_coral_height)
        // .andThen(new WaitCommand(0.25))
        // .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
        // this.ARM_HALFWAY = () ->
        // elevator.moveToPosition(Constants.Elevator.L1_coral_height);
        this.INTAKE_CORAL = () -> manipulator.intakeCoral().alongWith(funnel.intakeCoral());
        this.STOP_INTAKE = () -> manipulator.stop().withTimeout(0.000001).alongWith(funnel.stop());
        // this.REEF_DRIVE_THEN_SCORE_L4 = (lefty) ->
        // drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty).until(drivetrain.reachedPose).withName("final alignment")
        // .andThen(SCORE_CORAL_L4.get().deadlineFor(drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty)));
    }

    private Command feedAndScorePoseL4(String pathName) {
        FollowPath path =
                new FollowPath(
                        pathName,
                        this.drivetrain::getPose,
                        this.drivetrain::followSample,
                        DriverStation.getAlliance().orElse(Alliance.Blue),
                        drivetrain);

        path.trajectory.sampleAt(0, false);

        return this.elevator
                .moveToPosition(Constants.Elevator.STOW_height)
                .until(this.elevator.isUp.negate())
                .andThen(
                        path.gimmeCommand()
                                .andThen(DriveCommands.stop(drivetrain))
                                .alongWith(
                                        this.intakeUntilIntooken()
                                                .andThen(
                                                        this.elevator.moveToPosition(
                                                                Constants.Elevator
                                                                        .L4_coral_height))))
                .andThen(new WaitCommand(Seconds.of(0.25)))
                .andThen(this.manipulator.outtakeCoral().withTimeout(THROW_TIME));
    }

    public Command oneCoralCenter() {
        // score on G
        Pose2d scorePose;
        if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.GH_B.getLeft(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.GH_R.getLeft(ElevatorSetbacks.L4);
        }

        DriveToPose driveToPose = new DriveToPose(drivetrain, () -> scorePose);

        return new WaitCommand(Seconds.of(5.0))
                .andThen(
                        driveToPose
                                .until(() -> driveToPose.atTarget(Inches.of(1.5), Degrees.of(3.0)))
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L4_coral_height))
                                .andThen(DriveCommands.stop(drivetrain))
                                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME))
                                .andThen(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.STOW_height)));
    }

    public Command intakeUntilIntooken() {
        if (Robot.isSimulation()) {
            return new PrintCommand("Intooken");
        }

        return this.INTAKE_CORAL
                .get()
                .until(this.manipulator.justIntookenGamePooken.and(this.elevator.isStowed))
                .andThen(this.STOP_INTAKE.get())
                .andThen(new WaitCommand(0.1));
    }

    public Command testAuto() {
        FollowPath f =
                new FollowPath(
                        "JL4ToLeftFeedToLL4",
                        this.drivetrain::getPose,
                        this.drivetrain::followSample,
                        DriverStation.getAlliance().orElse(Alliance.Blue),
                        drivetrain);
        f.trajectory.sampleAt(0, false);
        return oneCoralInsideLeft()
                .andThen(elevator.moveToPosition(Constants.Elevator.STOW_height))
                .andThen(f.gimmeCommand());
    }

    public Command oneCoralInsideLeft() {
        // score on J
        Pose2d scorePose;
        if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.IJ_B.getLeft(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.IJ_R.getLeft(ElevatorSetbacks.L4);
        }

        DriveToPose driveToPose = new DriveToPose(drivetrain, () -> scorePose);

        return driveToPose
                .until(() -> driveToPose.atTarget(Inches.of(1.5), Degrees.of(3.0)))
                .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(DriveCommands.stop(drivetrain))
                .andThen(new WaitCommand(Seconds.of(0.125)))
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
    }

    public Command oneCoralInsideRight() {
        // score on E
        Pose2d scorePose;
        if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.EF_B.getRight(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.EF_R.getRight(ElevatorSetbacks.L4);
        }

        DriveToPose driveToPose = new DriveToPose(drivetrain, () -> scorePose);

        return driveToPose
                .until(() -> driveToPose.atTarget(Inches.of(1.5), Degrees.of(3.0)))
                .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(DriveCommands.stop(drivetrain))
                .andThen(new WaitCommand(Seconds.of(0.125)))
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
    }

    @SuppressWarnings("unused")
    public Command JtoLeftFeed() {
        FollowPath path =
                new FollowPath(
                        "JL4ToLeftFeed",
                        this.drivetrain::getPose,
                        this.drivetrain::followSample,
                        DriverStation.getAlliance().orElse(Alliance.Blue),
                        drivetrain);
        return path.gimmeCommand();
    }

    private Command EtoRightFeed() {
        FollowPath path =
                new FollowPath(
                        "EL4ToRightFeed",
                        this.drivetrain::getPose,
                        this.drivetrain::followSample,
                        DriverStation.getAlliance().orElse(Alliance.Blue),
                        drivetrain);
        return path.gimmeCommand();
    }

    public Command twoCoralInsideRight() {
        return oneCoralInsideRight().andThen(feedAndScorePoseL4("EL4ToRightFeedToCL4"));
    }

    public Command threeCoralInsideRight() {
        return twoCoralInsideRight().andThen(feedAndScorePoseL4("CL4ToRightFeedToDL4"));
    }

    public Command twoCoralInsideLeft() {
        return oneCoralInsideLeft().andThen(feedAndScorePoseL4("JL4ToLeftFeedToLL4"));
    }

    public Command threeCoralInsideLeft() {
        return twoCoralInsideLeft().andThen(feedAndScorePoseL4("LL4ToLeftFeedToKL4"));
    }

    public Command fourCoralInsideLeft() {
        return threeCoralInsideLeft().andThen(feedAndScorePoseL4("KL4ToLeftFeedToAL4_Special"));
    }

    // public Command oneCoralFeed() {
    // FollowPath jToFeed = new FollowPath("JL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // return oneCoral()
    // .andThen(jToFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get());
    // }

    // public Command hToFeed() {
    // FollowPath path = new FollowPath("HL4ToRightFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    // }

    // public Command threeCoral() {
    // FollowPath JL4ToLeftFeed = new FollowPath("JL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath LeftFeedToKL4 = new FollowPath("LeftFeedToKL4", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath KL4ToLeftFeed = new FollowPath("KL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath LeftFeedToLL4 = new FollowPath("LeftFeedToLL4", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);

    // return oneCoral()
    // .andThen(JL4ToLeftFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get())
    // .andThen(STOP_INTAKE.get())
    // .andThen(LeftFeedToKL4.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(SCORE_CORAL_L4.get())
    // .andThen(STOW_ARM.get())
    // .andThen(KL4ToLeftFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get())
    // .andThen(STOP_INTAKE.get())
    // .andThen(LeftFeedToLL4.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(SCORE_CORAL_L4.get())
    // .andThen(STOW_ARM.get());
    // }

    // public Command testScore() {
    // FollowPath testForward = new FollowPath("test_1", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath testToFeed = new FollowPath("test_2", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath feedToScore = new FollowPath("test_3", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);

    // return Commands.runOnce(() ->
    // this.drivetrain.resetPose(testForward.getInitialPose()))
    // .andThen(testForward.gimmeCommand());
    // }
}
