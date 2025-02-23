package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {

  private static final LoggedTunableNumberFactory factory =
      new LoggedTunableNumberFactory("Elevator");

  private static final LoggedTunableNumber kP = factory.getNumber("kP", REAL_P);
  private static final LoggedTunableNumber kI = factory.getNumber("kI", REAL_I);
  private static final LoggedTunableNumber kD = factory.getNumber("kD", REAL_D);

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  // TODO: This is more competant than the way its done in the hang arm
  private final WristVisualizer currentWristVisualizer =
      new WristVisualizer("Wrist/CurrentPosition", 64, 64, 32, 0, Color.kGreen);
  private final WristVisualizer targetWristVisualizer =
      new WristVisualizer("Wrist/SetpointPosition", 64, 64, 32, 0, Color.kOrange);

  /** Creates a new Template. */
  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    currentWristVisualizer.setRotations(inputs.position);
    targetWristVisualizer.setRotations(inputs.setpoint);

    Logger.processInputs("Wrist", inputs);
  }

  public void goTo(double setpoint) {
    io.goTo(setpoint);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  /** Get position in rotations */
  public double getPosition() {
    return inputs.position;
  }

  /** Get setpoint in rotations */
  public double getSetpoint() {
    return inputs.setpoint;
  }

  public void setPid(double kP, double kI, double kD) {
    io.setPid(kP, kI, kD);
  }
}
