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
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team9140.frc2025.commands.DriveCommands;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.helpers.AutoAiming;
import org.team9140.frc2025.subsystems.cantdle.Cantdle;
import org.team9140.frc2025.subsystems.climber.Climber;
import org.team9140.frc2025.subsystems.climber.ClimberIO;
import org.team9140.frc2025.subsystems.climber.ClimberIOSim;
import org.team9140.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.team9140.frc2025.subsystems.drive.Drive;
import org.team9140.frc2025.subsystems.drive.GyroIO;
import org.team9140.frc2025.subsystems.drive.GyroIOPigeon2;
import org.team9140.frc2025.subsystems.drive.ModuleIO;
import org.team9140.frc2025.subsystems.drive.ModuleIOSim;
import org.team9140.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.team9140.frc2025.subsystems.elevator.Elevator;
import org.team9140.frc2025.subsystems.elevator.ElevatorIO;
import org.team9140.frc2025.subsystems.elevator.ElevatorIOSim;
import org.team9140.frc2025.subsystems.elevator.ElevatorIOTalonFX;
import org.team9140.frc2025.subsystems.funnel.Funnel;
import org.team9140.frc2025.subsystems.manipulator.Manipulator;
import org.team9140.frc2025.subsystems.vision.Vision;
import org.team9140.frc2025.subsystems.vision.VisionIO;
import org.team9140.frc2025.subsystems.vision.VisionIOLimelight;
import org.team9140.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import org.team9140.lib.rollers.RollerIO;
import org.team9140.lib.rollers.RollerIOSim;
import org.team9140.lib.rollers.RollerIOTalonFX;
import org.team9140.lib.rollers.RollerIOTalonSRX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    @SuppressWarnings("unused")
    private final Vision vision;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> autoChooser;

    private final Climber climber;
    private final Drive drive;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Funnel funnel;
    private final Cantdle cantdle = Cantdle.getInstance();

    private final Trigger stickInput =
            new Trigger(
                    () ->
                            Math.abs(this.controller.getLeftX()) > 0.35
                                    || Math.abs(this.controller.getLeftY()) > 0.35
                                    || Math.abs(this.controller.getRightX()) > 0.35);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOLimelight(Constants.Vision.limeA, drive::getRotation),
                                new VisionIOLimelight(Constants.Vision.limeB, drive::getRotation),
                                new VisionIOLimelight(Constants.Vision.limeC, drive::getRotation));

                elevator = new Elevator(new ElevatorIOTalonFX());

                manipulator =
                        new Manipulator(new RollerIOTalonSRX(Constants.Ports.MANIPULATOR_MOTOR));

                funnel =
                        new Funnel(
                                new RollerIOTalonFX(
                                        Constants.Ports.FUNNEL_MOTOR,
                                        "sigma",
                                        Constants.Funnel.STATOR_LIMIT.in(Amps),
                                        Constants.Funnel.SUPPLY_LIMIT.in(Amps)));

                climber = new Climber(new ClimberIOTalonFX());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(
                                        Constants.Vision.limeA,
                                        Constants.Vision.robotToLimeA,
                                        drive::getPose),
                                new VisionIOPhotonVisionSim(
                                        Constants.Vision.limeB,
                                        Constants.Vision.robotToLimeB,
                                        drive::getPose),
                                new VisionIOPhotonVisionSim(
                                        Constants.Vision.limeC,
                                        Constants.Vision.robotToLimeC,
                                        drive::getPose));

                elevator = new Elevator(new ElevatorIOSim());

                // moi is made up tbh, and the gear ratio is weird cuz it drives two different
                // rods at different gear ratios. lowkey sim is unnecessary for this
                manipulator =
                        new Manipulator(
                                new RollerIOSim(
                                        DCMotor.getAndymarkRs775_125(1), 0.004, 1.96 / 0.48));

                // Again, don't know the moi, could calculate later but sim ain't too important
                // for this
                funnel =
                        new Funnel(new RollerIOSim(DCMotor.getKrakenX60Foc(1), 0.004, 2.4 / 0.885));

                climber = new Climber(new ClimberIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});

                elevator = new Elevator(new ElevatorIO() {});

                manipulator = new Manipulator(new RollerIO() {});

                funnel = new Funnel(new RollerIO() {});

                climber = new Climber(new ClimberIO() {});

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        controller::getLeftY,
                        controller::getLeftX,
                        () -> -controller.getRightX()));

        controller
                .rightTrigger()
                .onTrue(this.manipulator.outtake().until(this.controller.rightTrigger().negate()));

        controller
                .rightBumper()
                .and(elevator.isStowed)
                .whileTrue(
                        this.manipulator
                                .intakeCoral()
                                .alongWith(this.funnel.intakeCoral())
                                .withName("intake coral"));
        controller
                .leftBumper()
                .whileTrue(
                        this.manipulator
                                .unstickCoral()
                                .alongWith(this.funnel.unstickCoral())
                                .withName("unstick coral"));

        this.controller
                .rightBumper()
                .and(elevator.isStowed.negate())
                .whileTrue(this.manipulator.intakeAlgae().withName("intake algae"));

        this.controller
                .y()
                .and(this.controller.povRight())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L4, false)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L4_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("high coral R"));

        this.controller
                .y()
                .and(this.controller.povLeft())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L4, true)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L4_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("high coral L"));

        this.controller
                .y()
                .and(this.controller.povCenter())
                .and(this.manipulator.hasAlgae)
                .onTrue(
                        this.elevator
                                .moveToPosition(Constants.Elevator.NET_HEIGHT)
                                .withName("net height"));

        this.controller
                .b()
                .and(this.controller.povRight())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L3, false)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L3_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("highish (level 3) coral R"));

        this.controller
                .b()
                .and(this.controller.povLeft())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L3, true)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L3_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("highish (level 3) coral L"));

        this.controller
                .b()
                .and(this.controller.povCenter())
                .onTrue(
                        this.elevator
                                .moveToPosition(Constants.Elevator.L3_ALGAE_height)
                                .until(this.stickInput)
                                .withName("highish (level 3) algae center"));

        this.elevator
                .isAlgaeing
                .and(this.controller.rightBumper())
                .onTrue(
                        this.drive
                                .algaeReefDrive()
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                () ->
                                                        AutoAiming.getClosestFace(
                                                                        this.drive
                                                                                .getPose()
                                                                                .getTranslation())
                                                                .getAlgaeElevatorHeight()))
                                .until(this.stickInput)
                                .withName("autoaiming algae"));

        this.controller
                .a()
                .and(this.controller.povRight())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L2, false)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L2_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("mid coral R"));

        this.controller
                .a()
                .and(this.controller.povLeft())
                .onTrue(
                        this.drive
                                .coralReefDrive(Constants.ElevatorSetbacks.L2, true)
                                .alongWith(
                                        this.elevator.moveToPosition(
                                                Constants.Elevator.L2_coral_height))
                                .alongWith(
                                        this.cantdle.blinkColorForever(
                                                Cantdle.PURPLE, Seconds.of(0.5)))
                                .until(this.stickInput)
                                .withName("mid coral L"));

        this.controller
                .a()
                .and(this.controller.povCenter())
                .onTrue(
                        this.elevator
                                .moveToPosition(Constants.Elevator.L2_ALGAE_height)
                                .until(this.stickInput)
                                .withName("mid (level 2) algae center"));

        this.controller.x().onTrue(this.elevator.setGoal(Constants.Elevator.STOW_height));

        this.controller
                .x()
                .whileTrue(
                        this.climber.climb(
                                this.controller.getHID()::getLeftTriggerAxis,
                                this.controller.getHID()::getRightTriggerAxis));
        this.controller.back().onTrue(this.climber.prep());
        // this.elevator.isUp.onTrue(this.drive.engageSlowMode())
        // .onFalse(this.drivetrain.disengageSlowMode());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
