package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {

  private static final LoggedTunableNumberFactory factory = new LoggedTunableNumberFactory("Wrist");

  private static final LoggedTunableNumber kP = factory.getNumber("kP", WRIST_P);
  private static final LoggedTunableNumber kI = factory.getNumber("kI", WRIST_I);
  private static final LoggedTunableNumber kD = factory.getNumber("kD", WRIST_D);

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Creates a new Template. */
  public Wrist(WristIO io) {
    this.io = io;

    io.setPid(kP.get(), kI.get(), kD.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> io.setPid(values[0], values[1], values[2]), kP, kI, kD);
  }

  public void goTo(double setpoint) {
    io.runPosition(setpoint);
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
}
