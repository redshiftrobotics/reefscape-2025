package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HangVisualization {
  private final String name;

  private final LoggedMechanism2d mechanism;

  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d arm;

  public HangVisualization(String name, Color color) {
    this.name = name;

    mechanism =
        new LoggedMechanism2d(
            Units.feetToMeters(4.0), Units.feetToMeters(4.0), new Color8Bit(Color.kDarkGray));

    root = mechanism.getRoot(name + " Root", Units.feetToMeters(2.0), Units.feetToMeters(2.0));

    arm =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Hang", Units.feetToMeters(1.5), 0, 8, new Color8Bit(color)));
  }

  public void update(Rotation2d angle) {

    arm.setAngle(angle.plus(Rotation2d.kCCW_90deg));

    Logger.recordOutput("HangVisualizer/" + name, mechanism);
  }
}
