package org.team9140.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class Util {
  private static DriverStation.Alliance alliance = null;

  private static final double defaultDeadband = 0.12;
  private static final double EPSILON = 0.00000001;

  public static double applyDeadband(double in, double deadband) {
    if (Math.abs(in) < deadband) {
      return 0.0;
    } else if (in > 0) {
      return (in - deadband) / (1.0 - deadband);
    } else {
      return (in + deadband) / (1.0 - deadband);
    }
  }

  public static double applyDeadband(double in) {
    return applyDeadband(in, defaultDeadband);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  public static final Distance TRANSLATION_E = Inches.of(1.5);
  public static final Angle ROTATION_E = Degrees.of(2);

  public static boolean epsilonEquals(Pose2d a, Pose2d b) {
    boolean transValid =
        a.getTranslation().getDistance(b.getTranslation()) < TRANSLATION_E.in(Meters);
    boolean rotValid =
        rotationEpsilonEquals(a.getRotation(), b.getRotation(), ROTATION_E.in(Radians));

    return transValid && rotValid;
  }

  public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b, double epsilon) {
    return Math.abs(MathUtil.angleModulus(a.getRadians() - b.getRadians())) <= epsilon;
  }

  public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b) {
    return rotationEpsilonEquals(a, b, Math.toRadians(5.0));
  }

  public static double clamp(double val, double limit) {
    if (val > limit) {
      return limit;
    } else return Math.max(val, -limit);
  }

  public static void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(null);
  }

  public static Optional<DriverStation.Alliance> getAlliance() {
    if (alliance == null) updateAlliance();
    return Optional.ofNullable(alliance);
  }
}
