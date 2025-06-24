package org.team9140.frc2025.subsystems.cantdle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class CantdleVisualizer {
    private final LoggedMechanism2d mechanism =
            new LoggedMechanism2d(Units.inchesToMeters(2.0), Units.inchesToMeters(6.0));
    private final LoggedMechanismLigament2d light;

    public CantdleVisualizer() {
        this.light =
                mechanism
                        .getRoot("Cantldle Root", Units.inchesToMeters(0), Units.inchesToMeters(20))
                        .append(
                                new LoggedMechanismLigament2d(
                                        "Cantdle", Units.inchesToMeters(2.0), 0));
    }

    public void update(Color8Bit color) {
        this.light.setColor(color);
        Logger.recordOutput("Mechanism2d/Cantdle", mechanism);
    }
}
