package org.team9140.frc2025.subsystems.funnel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2025.Constants;
import org.team9140.lib.rollers.RollerIO;
import org.team9140.lib.rollers.RollerIOInputsAutoLogged;

public class Funnel extends SubsystemBase {
  RollerIO motor;
  RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Funnel(RollerIO motor) {
    this.motor = motor;

    this.motor.setBrakeMode(true);
    this.motor.setInverted(false);

    this.motor.stop();

    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
  }

  public Command stop() {
    return this.runOnce(() -> this.motor.stop()).withName("off");
  }

  public Command intakeCoral() {
    return this.setVoltage(Constants.Funnel.INTAKE_VOLTAGE).withName("intake coral");
  }

  public Command setVoltage(Voltage v) {
    return this.run(() -> this.motor.runVolts(v.in(Volts)));
  }

  public Command unstickCoral() {
    return this.setVoltage(Constants.Funnel.UNSTICK_VOLTAGE).withName("unstick coral");
  }
}
