package org.team9140.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.team9140.frc2025.Constants;

public class ElevatorVisualizer {
  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(Units.inchesToMeters(20), Units.feetToMeters(7.0));
  private final LoggedMechanismLigament2d elevator;

  public ElevatorVisualizer(String name) {
    this.name = name;
    elevator =
        mechanism
            .getRoot(name + " Root", 0, 0)
            .append(
                new LoggedMechanismLigament2d(
                    name + " Elevator",
                    Units.inchesToMeters(26.0),
                    Constants.Elevator.ElevatorAngle.in(Degrees)));
  }

  public void update(double elevatorHeightMeters) {
    elevator.setLength(elevatorHeightMeters);
    Logger.recordOutput("Mechanism2d/" + name, mechanism);
  }
}
