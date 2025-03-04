package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private final String name;

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(9.0), new Color8Bit(Color.kDarkGray));

  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d carriageMechanism;
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismLigament2d coralWristLigament;
  private final LoggedMechanismLigament2d algaeWristLigament;
  private final LoggedMechanismLigament2d algaeIntake;
  private final LoggedMechanismLigament2d coralIntake;

  public SuperstructureVisualizer(String name, Color color) {
    this.name = name;

    root =
        mechanism.getRoot(
            name + " Root",
            Units.inchesToMeters(34.886649) / 2,
            Units.inchesToMeters(34.886649 - 6));

    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator", Units.inchesToMeters(26.0), 90, 4.0, new Color8Bit(color)));

    carriageMechanism =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Carriage",
                ElevatorConstants.carriageHeight,
                0,
                4.0,
                new Color8Bit(color)));

    algaeWristLigament =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Coral Wrist", Units.inchesToMeters(10.0), 0, 4.0, new Color8Bit(color)));

    coralWristLigament =
        carriageMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Algae Wrist", Units.inchesToMeters(9.0), 0, 4.0, new Color8Bit(color)));

    algaeIntake =
        algaeWristLigament.append(
            new LoggedMechanismLigament2d(
                name + " Algae Intake", 0, 0, 10.0, new Color8Bit(color)));

    coralIntake =
        coralWristLigament.append(
            new LoggedMechanismLigament2d(
                name + " Coral Intake", 0, 0, 10.0, new Color8Bit(color)));
  }

  public void update(
      double carriageHeight,
      double wristRotations,
      double algaeRotations,
      boolean coralIntakeRunning,
      boolean algaeIntakeRunning) {
    elevatorMechanism.setLength(carriageHeight);
    coralWristLigament.setAngle(Units.rotationsToDegrees(wristRotations));
    algaeWristLigament.setAngle(Units.rotationsToDegrees(algaeRotations));

    coralIntake.setLineWeight(coralIntakeRunning ? 10.0 : 0.0);
    algaeIntake.setLineWeight(algaeIntakeRunning ? 10.0 : 0.0);

    Logger.recordOutput("SuperstructureVisualizer/" + name, mechanism);
  }
}
