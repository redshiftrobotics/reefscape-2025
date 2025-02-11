package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class WristVisualizer {
  private final String name;

  private final LoggedMechanism2d mech =
      new LoggedMechanism2d(
          WristConstants.ARM_LENGTH, Units.inchesToMeters(50), new Color8Bit(Color.kBlack));
  private final LoggedMechanismLigament2d wrist;

  public WristVisualizer(String name, Color color) {
    this.name = name;

    LoggedMechanismRoot2d root = mech.getRoot(name + " Root", 0.1, 0.06);

    wrist =
        root.append(
            new LoggedMechanismLigament2d(
                name + "Wrist", WristConstants.ARM_LENGTH, 0, 4.0, new Color8Bit(color)));
  }

  public void update(Rotation2d rot) {
    wrist.setAngle(rot);
    Logger.recordOutput("WristMechanism2d/" + name, mech);
  }
}
