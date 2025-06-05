package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.wrist.WristConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private final String name;

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));

  public static final double ROOT_X = Units.inchesToMeters(34.886649) * 1.0 / 4.0;
  public static final double ROOT_Y = Units.inchesToMeters(3);

  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismLigament2d carriage, carriageJoint;
  private final LoggedMechanismLigament2d coralWristLigament;
  private final LoggedMechanismLigament2d coralIntake;

  public SuperstructureVisualizer(String name, Color color) {
    this.name = name;

    root = mechanism.getRoot(name + " Root", ROOT_X, ROOT_Y);

    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator", Units.inchesToMeters(26.0), 90, 4.0, new Color8Bit(color)));

    carriage =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Carriage",
                ElevatorConstants.carriageHeight,
                0,
                4.0,
                new Color8Bit(color)));

    carriageJoint =
        carriage.append(
            new LoggedMechanismLigament2d(
                name + " Carriage Joint",
                Units.inchesToMeters(2),
                90.0,
                4.0,
                new Color8Bit(color)));

    coralWristLigament =
        carriageJoint.append(
            new LoggedMechanismLigament2d(
                name + " Algae Wrist", WristConstants.WRIST_LENGTH, 0, 4.0, new Color8Bit(color)));

    coralIntake =
        coralWristLigament.append(
            new LoggedMechanismLigament2d(
                name + " Coral Intake", 0.0, -90, 10.0, new Color8Bit(235, 137, 52)));
  }

  public void update(double carriageHeight, double wristRadians, double coralIntakeOutput) {
    elevatorMechanism.setLength(carriageHeight + ElevatorConstants.elevatorDistanceFromGround);
    coralWristLigament.setAngle(new Rotation2d(-wristRadians));

    coralIntake.setLength(coralIntakeOutput * Units.inchesToMeters(3.0));

    Logger.recordOutput("SuperstructureVisualizer/" + name, mechanism);
  }
}
