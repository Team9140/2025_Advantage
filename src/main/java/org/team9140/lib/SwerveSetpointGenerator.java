package org.team9140.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.team9140.frc2025.Constants;

/**
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all of the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 *
 * <p>Modified from team 254's 2023 code.
 * https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/swerve/SwerveSetpointGenerator.java
 */
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics mKinematics;
    private final Translation2d[] modulePositions;

    // 6000 RPM / 60 sec/min / 150 * 7 ~ 4.5 module rotations per second, do less
    private static final AngularVelocity kMaxSteeringVelocity = RotationsPerSecond.of(4);

    // run up to full speed in a quarter second??
    private static final LinearAcceleration kMaxDriveAcceleration =
            Constants.Drive.MAX_teleop_velocity.div(Seconds.of(0.25));

    public SwerveSetpointGenerator(
            final SwerveDriveKinematics kinematics, Translation2d[] modules) {
        this.mKinematics = kinematics;
        this.modulePositions = modules;
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
     * direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e.
     *     prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive
     *     direction.
     */
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
     * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
     * while being robust in ways that e.g. the Newton-Raphson method isn't.
     *
     * @param func The Function2d to take the root of.
     * @param x_0 x value of the lower bracket.
     * @param y_0 y value of the lower bracket.
     * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param x_1 x value of the upper bracket.
     * @param y_1 y value of the upper bracket.
     * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
     *     (approximate) root.
     */
    private double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left) {
        if (iterations_left < 0 || Util.epsilonEquals(f_0, f_1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess)
                            * findRoot(
                                    func,
                                    x_guess,
                                    y_guess,
                                    f_guess,
                                    x_1,
                                    y_1,
                                    f_1,
                                    iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess
                    * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    protected double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_deviation,
            int max_iterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func =
                (x, y) -> {
                    return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
                };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    protected double findDriveMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_vel_step,
            int max_iterations) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func =
                (x, y) -> {
                    return Math.hypot(x, y) - offset;
                };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    private boolean isStopped(ChassisSpeeds cs) {
        return Math.abs(cs.omegaRadiansPerSecond) <= Math.toRadians(1.0)
                && Math.abs(cs.vxMetersPerSecond) <= 0.01
                && Math.abs(cs.vyMetersPerSecond) <= 0.01;
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
     *     iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path
     *     following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
     *     desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredState, double dt) {
        SwerveModuleState[] desiredModuleState = mKinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.

        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredModuleState, Constants.Drive.MAX_teleop_velocity);
        desiredState = mKinematics.toChassisSpeeds(desiredModuleState);

        // Special case: desiredState is a complete stop. In this case, module angle is
        // arbitrary, so just use the previous angle.
        boolean need_to_steer = true;
        if (isStopped(desiredState)) {
            need_to_steer = false;
            for (int i = 0; i < modulePositions.length; ++i) {
                desiredModuleState[i].angle = prevSetpoint.mModuleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = new double[modulePositions.length];
        double[] prev_vy = new double[modulePositions.length];
        Rotation2d[] prev_heading = new Rotation2d[modulePositions.length];
        double[] desired_vx = new double[modulePositions.length];
        double[] desired_vy = new double[modulePositions.length];
        Rotation2d[] desired_heading = new Rotation2d[modulePositions.length];
        boolean all_modules_should_flip = true;
        for (int i = 0; i < modulePositions.length; ++i) {
            prev_vx[i] =
                    prevSetpoint.mModuleStates[i].angle.getCos()
                            * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
            prev_vy[i] =
                    prevSetpoint.mModuleStates[i].angle.getSin()
                            * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
            prev_heading[i] = prevSetpoint.mModuleStates[i].angle;
            if (prevSetpoint.mModuleStates[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = prev_heading[i].plus(new Rotation2d(Degrees.of(180.0)));
            }
            desired_vx[i] =
                    desiredModuleState[i].angle.getCos()
                            * desiredModuleState[i].speedMetersPerSecond;
            desired_vy[i] =
                    desiredModuleState[i].angle.getSin()
                            * desiredModuleState[i].speedMetersPerSecond;
            desired_heading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desired_heading[i] = desired_heading[i].plus(new Rotation2d(Degrees.of(180.0)));
            }
            if (all_modules_should_flip) {
                double required_rotation_rad =
                        Math.abs(
                                prev_heading[i]
                                        .unaryMinus()
                                        .rotateBy(desired_heading[i])
                                        .getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip
                && !isStopped(prevSetpoint.mChassisSpeeds)
                && !isStopped(desiredState)) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to
            // the complement of the desired
            // angle, and accelerate again.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the
        // start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that
        // no kinematic limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.mChassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.mChassisSpeeds.vyMetersPerSecond;
        double dtheta =
                desiredState.omegaRadiansPerSecond
                        - prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1,
        // we are at desiredState.
        double min_s = 1.0;

        // In cases where an individual module is stopped, we want to remember the right
        // steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modulePositions.length);
        // Enforce steering velocity limits. We do this by taking the derivative of
        // steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states.
        // We remember the minimum across all modules, since
        // that is the active constraint.
        final double max_theta_step = dt * kMaxSteeringVelocity.in(RadiansPerSecond);
        for (int i = 0; i < modulePositions.length; ++i) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.mModuleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (Util.epsilonEquals(prevSetpoint.mModuleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final
                // steering angle, so limit based
                // purely on rotation in place.
                if (Util.epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.mModuleStates[i].angle));
                    continue;
                }

                var necessaryRotation =
                        prevSetpoint
                                .mModuleStates[i]
                                .angle
                                .unaryMinus()
                                .rotateBy(desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded =
                        Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(
                            i,
                            Optional.of(
                                    prevSetpoint.mModuleStates[i].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians())
                                                            * max_theta_step))));
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s =
                    findSteeringMaxS(
                            prev_vx[i],
                            prev_vy[i],
                            prev_heading[i].getRadians(),
                            desired_vx[i],
                            desired_vy[i],
                            desired_heading[i].getRadians(),
                            max_theta_step,
                            kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        // Enforce drive wheel acceleration limits.
        final double max_vel_step = dt * kMaxDriveAcceleration.in(MetersPerSecondPerSecond);
        for (int i = 0; i < modulePositions.length; ++i) {
            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }
            double vx_min_s =
                    min_s == 1.0
                            ? desired_vx[i]
                            : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
            double vy_min_s =
                    min_s == 1.0
                            ? desired_vy[i]
                            : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and
            // min_s, because we already know we can't go faster
            // than that.
            final int kMaxIterations = 10;
            double s =
                    min_s
                            * findDriveMaxS(
                                    prev_vx[i],
                                    prev_vy[i],
                                    Math.hypot(prev_vx[i], prev_vy[i]),
                                    vx_min_s,
                                    vy_min_s,
                                    Math.hypot(vx_min_s, vy_min_s),
                                    max_vel_step,
                                    kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds =
                new ChassisSpeeds(
                        prevSetpoint.mChassisSpeeds.vxMetersPerSecond + min_s * dx,
                        prevSetpoint.mChassisSpeeds.vyMetersPerSecond + min_s * dy,
                        prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
        var retStates = mKinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < modulePositions.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation =
                    prevSetpoint.mModuleStates[i].angle.unaryMinus().rotateBy(retStates[i].angle);
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.plus(new Rotation2d(Degrees.of(180.0)));
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }

    public static class SwerveSetpoint {
        public ChassisSpeeds mChassisSpeeds;
        public SwerveModuleState[] mModuleStates;

        public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
            this.mChassisSpeeds = chassisSpeeds;
            this.mModuleStates = initialStates;
        }

        @Override
        public String toString() {
            String ret = mChassisSpeeds.toString() + "\n";
            for (int i = 0; i < mModuleStates.length; ++i) {
                ret += "  " + mModuleStates[i].toString() + "\n";
            }
            return ret;
        }
    }
}
