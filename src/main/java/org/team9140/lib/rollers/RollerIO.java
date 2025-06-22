package org.team9140.lib.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  class RollerIOInputs {
    public RollerIOData data =
        new RollerIOData(0.0, 0.0, 0.0, 0.0, false, false);
  }

  record RollerIOData(
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean tempFault,
      boolean connected) {}

  default void updateInputs(RollerIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setBrakeMode(boolean brake) {}

  default void setInverted(boolean inverted) {}
}
