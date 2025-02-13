package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HangVisualization {
  private final String key;

  private double angle;

  private LoggedMechanism2d mechanism2d;
  private LoggedMechanismRoot2d armPivot;
  private LoggedMechanismLigament2d arm;

  public HangVisualization(
      String key, int width, int height, int length, double initialPositionRadians, Color color) {
    this.key = key;

    mechanism2d = new LoggedMechanism2d(width, height);
    armPivot = mechanism2d.getRoot("ArmPivot", width / 2, height / 2);
    arm =
        armPivot.append(
            new LoggedMechanismLigament2d(
                "Simulated Arm", length, initialPositionRadians, 8, new Color8Bit(color)));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angle);
  }

  public void setAngle(Rotation2d angle) {
    this.angle = angle.getDegrees();

    arm.setAngle(angle);

    Logger.recordOutput(key, mechanism2d);
  }
}
