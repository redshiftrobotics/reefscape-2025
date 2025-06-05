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

  public static final double ROOT_X = Units.inchesToMeters(34.886649) * 3.0 / 4.0;
  public static final double ROOT_Y = Units.inchesToMeters(7);

  public HangVisualization(String name, Color color) {
    this.name = name;

    mechanism =
        new LoggedMechanism2d(
            Units.inchesToMeters(28.0), Units.feetToMeters(3.0), new Color8Bit(Color.kDarkGray));

    root = mechanism.getRoot(name + " Root", ROOT_X, ROOT_Y);

    arm =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Hang", Units.feetToMeters(1.5), 0, 6, new Color8Bit(color)));
  }

  public void update(Rotation2d angle) {

    arm.setAngle(angle.plus(Rotation2d.kCCW_90deg));

    Logger.recordOutput("HangVisualizer/" + name, mechanism);
  }
}
