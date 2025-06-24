package org.team9140.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public ClimberIOData data =
                new ClimberIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ClimberIOData(
            boolean motorConnected,
            boolean followerConnected,
            double positionRad,
            double velocityRadPerSec,
            double appliedVolts,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius,
            double followerAppliedVolts,
            double followerTorqueCurrentAmps,
            double followerSupplyCurrentAmps,
            double followerTempCelsius) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}
}
