package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private final String name;

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));

  private final LoggedMechanismLigament2d elevatorMechanism;

  private final LoggedMechanismLigament2d wristLigament;

  public SuperstructureVisualizer(String name, Color color) {
    this.name = name;

    LoggedMechanismRoot2d root = mechanism.getRoot(name + " Root", 0.1, 0.06);

    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator", Units.inchesToMeters(26.0), 90, 4.0, new Color8Bit(color)));

    wristLigament =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Wrist", Units.inchesToMeters(8), 0, 4.0, new Color8Bit(color)));
  }

  public void update(double carriageHeight, double wristRotations) {
    elevatorMechanism.setLength(carriageHeight);
    wristLigament.setAngle(Units.rotationsToDegrees(wristRotations));

    Logger.recordOutput("Mechanism2d/" + name, mechanism);
  }
}
