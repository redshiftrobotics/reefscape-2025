package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {
  private static final LoggedTunableNumberFactory factory =
      new LoggedTunableNumberFactory("Elevator");

  private static final frc.robot.utility.records.PIDConstants defaultPid =
      WristConstants.getPidConstants();

  private static final LoggedTunableNumber kP = factory.getNumber("kP", defaultPid.kP());
  private static final LoggedTunableNumber kI = factory.getNumber("kI", defaultPid.kI());
  private static final LoggedTunableNumber kD = factory.getNumber("kD", defaultPid.kD());

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setPid(values[0], values[1], values[2]);
        },
        kP,
        kI,
        kD);
  }

  /** In rotations. */
  public void setGoal(double setpoint) {
    io.runPosition(setpoint);
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.setpointRotations, inputs.positionRotations, TOLERANCE);
  }

  /** Get position in rotations */
  public double getMeasuredPosition() {
    return inputs.positionRotations;
  }

  /** Get setpoint in rotations */
  public double getSetpoint() {
    return inputs.setpointRotations;
  }

  public void setPid(double kP, double kI, double kD) {
    io.setPid(kP, kI, kD);
  }
}
