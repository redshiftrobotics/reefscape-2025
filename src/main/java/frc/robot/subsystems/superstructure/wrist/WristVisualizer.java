package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class WristVisualizer {
  private final String key;

  private double rotations;

  private LoggedMechanism2d mechanism2d;
  private LoggedMechanismRoot2d wristPivot;
  private LoggedMechanismLigament2d arm;

  public WristVisualizer(
      String key, int width, int height, int length, double initialRotations, Color color) {
    this.key = key;

    mechanism2d = new LoggedMechanism2d(width, height);
    wristPivot = mechanism2d.getRoot("WristPivot", width / 2, height / 2);
    arm =
        wristPivot.append(
            new LoggedMechanismLigament2d(
                "SimulatedWrist", length, initialRotations, 8, new Color8Bit(color)));
  }

  public double getRotations() {
    return rotations;
  }

  public void setRotations(double rotations) {
    this.rotations = rotations;

    arm.setAngle(Units.rotationsToDegrees(rotations));

    Logger.recordOutput(key, mechanism2d);
  }
}
