package org.team9140.frc2025.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.subsystems.drive.Drive;

// Heavily inspired by Team Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/commands/DriveToPose.java#L287
public class DriveToPose extends Command {
    // TODO: Change this from using drive to having a supplier of pose2ds and
    // consumer of chassisspeeds
    private final Drive drive;
    private final Supplier<Pose2d> target;
    private final TrapezoidProfile driveProfile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            Constants.Drive.AUTO_MAX_TRANSLATIONAL_VELOCITY.in(MetersPerSecond),
                            Constants.Drive.AUTO_MAX_TRANSLATIONAL_ACCELERATION.in(
                                    MetersPerSecondPerSecond)));

    private PhoenixPIDController translationalController =
            new PhoenixPIDController(
                    Constants.Drive.TRANSLATE_CONTROLLER_P,
                    Constants.Drive.TRANSLATE_CONTROLLER_I,
                    Constants.Drive.TRANSLATE_CONTROLLER_D);
    private ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    Constants.Drive.HEADING_CONTROLLER_P,
                    Constants.Drive.HEADING_CONTROLLER_I,
                    Constants.Drive.HEADING_CONTROLLER_D,
                    new TrapezoidProfile.Constraints(
                            Constants.Drive.AUTO_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                            Constants.Drive.AUTO_MAX_ANGULAR_ACCELERATION.in(
                                    RadiansPerSecondPerSecond)),
                    Constants.LOOP_PERIOD.in(Seconds));

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastSetpointRotation = Rotation2d.kZero;

    private double lastTime = 0.0;

    public DriveToPose(Drive drive, Supplier<Pose2d> target) {
        this.addRequirements(drive);

        this.drive = drive;
        this.target = target;

        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = target.get();
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        drive.getChassisSpeeds(), drive.getRotation());
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        translationalController.reset();
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointVelocity = linearFieldVelocity;
        lastSetpointRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = target.get();

        Pose2d poseError = currentPose.relativeTo(targetPose);
        double driveErrorAbs = poseError.getTranslation().getNorm();
        double thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

        Vector<N2> direction =
                targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
        // Essentially lastSetpointVelocity * cos(theta) where theta is the angle
        // between direction and lastSetpointVelocity
        double setpointVelocity = lastSetpointVelocity.toVector().dot(direction) / direction.norm();
        TrapezoidProfile.State driveSetpoint =
                driveProfile.calculate(
                        Constants.LOOP_PERIOD.in(Seconds),
                        new TrapezoidProfile.State(
                                direction.norm(),
                                -setpointVelocity), // Use negative as profile has zero at target
                        new TrapezoidProfile.State(0.0, 0.0));
        double driveVelocityScalar =
                translationalController.calculate(
                                driveErrorAbs,
                                driveSetpoint.position,
                                Utils.getCurrentTimeSeconds())
                        + driveSetpoint.velocity;
        Rotation2d targetToCurrentAngle =
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
        lastSetpointTranslation =
                new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
                        .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
                        .getTranslation();
        lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

        // Calculate theta speed
        double thetaSetpointVelocity =
                Math.abs((targetPose.getRotation().minus(lastSetpointRotation)).getDegrees()) < 10.0
                        ? (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                                / (Timer.getTimestamp() - lastTime)
                        : thetaController.getSetpoint().velocity;
        double thetaVelocity =
                thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                new TrapezoidProfile.State(
                                        targetPose.getRotation().getRadians(),
                                        thetaSetpointVelocity))
                        + thetaController.getSetpoint().velocity;
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
        lastSetpointRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();

        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        drive.getChassisSpeeds(), drive.getRotation());
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        // Command speeds
        if (drive != null) {
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            thetaVelocity,
                            currentPose.getRotation()));
        }

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
        Logger.recordOutput(
                "DriveToPose/VelocityMeasured",
                -linearFieldVelocity
                                .toVector()
                                .dot(
                                        targetPose
                                                .getTranslation()
                                                .minus(currentPose.getTranslation())
                                                .toVector())
                        / driveErrorAbs);
        Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput(
                "DriveToPose/Setpoint",
                new Pose2d[] {
                    new Pose2d(
                            lastSetpointTranslation,
                            Rotation2d.fromRadians(thetaController.getSetpoint().position))
                });
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
    }

    @Override
    public void end(boolean interrupted) {
        if (drive != null) drive.stop();
    }

    public boolean atTarget(Distance translationTolerance, Angle rotationTolerance) {
        Pose2d error = drive.getPose().relativeTo(target.get());
        return error.getTranslation().getNorm() < translationTolerance.in(Meters)
                && Math.abs(error.getRotation().getRadians()) < rotationTolerance.in(Radians);
    }
}
