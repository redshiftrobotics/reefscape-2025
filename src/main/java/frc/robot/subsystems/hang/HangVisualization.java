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

  private final LoggedMechanism2d mechanism2d;
  private final LoggedMechanismRoot2d armPivot;
  private final LoggedMechanismLigament2d arm;

  public HangVisualization(String name, Color color) {
    this.name = name;

    mechanism2d = new LoggedMechanism2d(Units.feetToMeters(4.0), Units.feetToMeters(4.0));
    armPivot =
        mechanism2d.getRoot(name + " Root", Units.feetToMeters(3.5), Units.feetToMeters(1.5));
    arm =
        armPivot.append(
            new LoggedMechanismLigament2d(
                name + " Hang", Units.feetToMeters(1), 0, 8, new Color8Bit(color)));
  }

  public void update(double angleRad) {

    arm.setAngle(Rotation2d.fromRadians(angleRad));

    Logger.recordOutput(name, mechanism2d);
  }
}
