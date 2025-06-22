// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.team9140.frc2025;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Time LOOP_PERIOD = Milliseconds.of(20.0);

  public static final Mass ROBOT_MASS = Pounds.of(100); // TODO: Put real value

  public static final LinearAcceleration G = MetersPerSecondPerSecond.of(9.807);

  public static class Ports {
    public static final int ELEVATOR_MOTOR_LEFT = 10;
    public static final int ELEVATOR_MOTOR_RIGHT = 11;
    public static final int MANIPULATOR_MOTOR = 4;
    public static final int FUNNEL_MOTOR = 13;
    public static final int CLIMBER_MOTOR_RIGHT = 14;
    public static final int CLIMBER_MOTOR_LEFT = 12;
    public static final int CANDLE_id = 0;
  }

  public static class Drive {
    public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.48);
    public static final LinearVelocity MAX_teleop_velocity = SPEED_AT_12_VOLTS.times(0.8);
    public static final AngularVelocity MAX_teleop_rotation = RotationsPerSecond.of(1);

    public static final LinearVelocity MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.06);
    public static final LinearVelocity MIN_TRANSLATIONAL_SPEED_TELEOP = MetersPerSecond.of(0.02);
    public static final AngularVelocity MIN_ROTATIONAL_SPEED = DegreesPerSecond.of(3);
    public static final AngularVelocity MIN_ROTATIONAL_SPEED_TELEOP = DegreesPerSecond.of(3);

    public static final double X_CONTROLLER_P = 2.5 * 3.141592653589793238462643383279502884197169399375;
    public static final double X_CONTROLLER_I = 0.0;
    public static final double X_CONTROLLER_D = 0.05;
    public static final double Y_CONTROLLER_P = X_CONTROLLER_P;
    public static final double Y_CONTROLLER_I = X_CONTROLLER_I;
    public static final double Y_CONTROLLER_D = X_CONTROLLER_D;
    public static final double HEADING_CONTROLLER_P = 8.0; // 8.0
    public static final double HEADING_CONTROLLER_I = 0.0;
    public static final double HEADING_CONTROLLER_D = 0.03; // 0.04

    public static final Time REACHEDPOSE_DEBOUNCE = Seconds.of(0.5);
    // TODO: Actual value
    public static final AngularVelocity MAX_MODULE_ROTATIONAL_SPEED = RotationsPerSecond.of(3.0);
  }

  public static class FieldItemPoses {
    public static final Pose2d REEF_BLUE = new Pose2d(4.48945, 4.0259, new Rotation2d());
    public static final Pose2d REEF_RED = new Pose2d(13.058775, 4.0259, new Rotation2d());
  }

  public static class Funnel {
    public static final Current STATOR_LIMIT = Amps.of(70);
    public static final Current SUPPLY_LIMIT = Amps.of(30);
    public static final Voltage INTAKE_VOLTAGE = Volts.of(12);
    public static final Voltage UNSTICK_VOLTAGE = Volts.of(-8);
  }

  public static class Manipulator {
    public static final double HOLD_VOLTAGE_ALGAE = 4.0;

    public static final double INTAKE_VOLTAGE_CORAL = 7.0;
    public static final double INTAKE_VOLTAGE_ALGAE = 10;

    public static final double OUTTAKE_VOLTAGE_CORAL = 8;
    public static final double OUTTAKE_VOLTAGE_ALGAE = -10;

    // public static final Measure<DistanceUnit> CORAL_DISTANCE =
    // Centimeters.of(10);
    // public static final Time INTAKE_CORAL_TIME = Seconds.of(1);

    // public static final int MIN_SIGNAL_STRENGTH = 5000;
    // public static final double PROXIMITY_HYSTERESIS = 0.05;
    // public static final double PROXIMITY_THRESHOLD = 0.4;
    // public static final double FORWARD_AUTOSET = 0.0;

    public static final Current MANIPULATOR_PEAK_CURRENT_LIMIT = Amps.of(30);
    public static final Time MANIPULATOR_PEAK_CURRENT_DURATION = Milliseconds.of(500.0);
    public static final Current MANIPULATOR_CONTINUOUS_CURRENT_LIMIT = Amps.of(20);
    public static final Current HOLD_AMPERAGE_GAME_PIECE = Amps.of(12.0);
    public static final Time INTOOKEN_TIME = Seconds.of(0.25);
  }

  public static final class Elevator {
    public static final Mass mass = Pounds.of(0.1);

    public static final Current STATOR_LIMIT = Amps.of(60.0);

    public static final double GEAR_RATIO = 60.0 / 12.0;
    public static final Distance SPOOL_RADIUS = Inches.of(0.75);
    public static final Distance SPOOL_CIRCUMFERENCE = SPOOL_RADIUS.times(Math.PI * 2.0);

    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond
        .of(Meters.of(2.5).div(SPOOL_CIRCUMFERENCE).magnitude());
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond
        .of(Meters.of(10.0).div(SPOOL_CIRCUMFERENCE).magnitude());

    public static Angle ElevatorAngle = Degrees.of(80);

    public static final Distance MIN_HEIGHT = Inches.of(0);
    public static final Distance MAX_HEIGHT = Inches.of(90);

    public static final Measure<DistanceUnit> POSITION_epsilon = Inches.of(0.75);

    public static Distance STOW_height = Inches.of(0.5);
    public static Distance L1_coral_height = Inches.of(24);
    public static Distance L2_coral_height = Inches.of(31);
    public static Distance L3_coral_height = Inches.of(47);
    public static Distance L4_coral_height = Inches.of(72.25);

    public static Distance L2_ALGAE_height = Inches.of(24);
    public static Distance L3_ALGAE_height = Inches.of(39);

    public static Distance NET_HEIGHT = Inches.of(80);

    public static Distance SOFT_LIMIT = Inches.of(81);

    // PID values for the real robot, not sim
    public static double kP = 150.0;
    public static double kI = 0.0;
    public static double kD = 12.0;

    // Feed Forwards
    public static double kS = 0.88302;
    public static double kV = 0.7863;
    public static double kA = 0.44435;
    public static double kG = 9.6;
  }

  public static final class Climber {
    public static final Current STATOR_LIMIT = Amps.of(80.0);
    public static final double GEAR_RATIO = 5.0 * 5.0 * 4.0 * 4.0;
    public static final Angle SOFT_LIMIT_LOWER = Radians.of(0.0);
    public static final Angle SOFT_LIMIT_HIGHER = Rotations.of(0.7);

    public static final Voltage MAX_OUTPUT = Volts.of(12.0);
    public static final Voltage PREP_VOLTAGE = Volts.of(10.0);
    public static final Angle PREPPED_POSITION = Rotations.of(0.46);
  }

  public enum ElevatorSetbacks {
    NONE(Inches.of(0)),
    L1(Inches.of(18.0)),
    L2(Inches.of(18.0)),
    L3(Inches.of(18.0 + 4.0)),
    L4(Inches.of(18.0 + 9.0)),
    ALGAE_L2(Inches.of(18.0)),
    ALGAE_L3(Inches.of(18.0 + 3.0));

    ElevatorSetbacks(Distance setback) {
      this.setbackinator = new Transform2d(setback.unaryMinus(), Meters.of(0), new Rotation2d());
    }

    private final Transform2d setbackinator;

    public Transform2d getSetbackinator() {
      return setbackinator;
    }
  }

  public static final class AutoAlign {
    public static final Distance REEF_RADIUS = Feet.of(5).plus(Inches.of(5.5)).div(2);

    public static final Transform2d HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER = new Transform2d(Meters.of(0),
        Inches.of(13).div(2), new Rotation2d());
  }

  public class Vision {
    // AprilTag layout
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static final String limeA = "limelight-a";
    public static final String limeB = "limelight-b";
    public static final String limeC = "limelight-c";

    // Robot to camera transforms for sim
    // (Not used by Limelight, configure in web UI instead)
    // TODO: Actual values
    public static final Transform3d robotToLimeA = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static final Transform3d robotToLimeB = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
    public static final Transform3d robotToLimeC = new Transform3d();

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] cameraStdDevFactors = new double[] {
        1.0, // LimeA
        1.0, // LimeB
        1.0 // LimeC
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
  }
}
