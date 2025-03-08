package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.records.ArmFeedForwardConstants;
import frc.robot.utility.records.PIDConstants;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {
  private final String name;

  private final LoggedTunableNumberFactory factory;
  private final LoggedTunableNumber kP, kI, kD;
  private final LoggedTunableNumber kS, kG, kV, kA;

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  // private Profil controller;
  private ArmFeedforward feedforward;

  private final Alert motorConnectedAlert;

  private Debouncer disabledDebouncer = new Debouncer(3, DebounceType.kRising);

  private double goalRotations = 0;

  /** Creates a new Wrist. */
  public Wrist(
      String name, WristIO io, PIDConstants feedback, ArmFeedForwardConstants feedforward) {
    this.name = name;
    this.io = io;

    factory = new LoggedTunableNumberFactory("Wrist " + name);
    kP = factory.getNumber("kP", feedback.kP());
    kI = factory.getNumber("kI", feedback.kI());
    kD = factory.getNumber("kD", feedback.kD());

    kS = factory.getNumber("kS", feedforward.kS());
    kG = factory.getNumber("kG", feedforward.kG());
    kV = factory.getNumber("kV", feedforward.kV());
    kA = factory.getNumber("kA", feedforward.kA());

    io.setPID(kP.get(), kI.get(), kD.get());
    io.setBrakeMode(true);

    this.feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    motorConnectedAlert =
        new Alert("Wrist " + name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist " + name, inputs);

    Logger.recordOutput(
        "Wrist" + name + "/degrees", Units.rotationsToDegrees(inputs.positionRotations));

    io.setBrakeMode(!disabledDebouncer.calculate(DriverStation.isDisabled()));

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setPID(values[0], values[1], values[2]);
        },
        kP,
        kI,
        kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> feedforward = new ArmFeedforward(values[0], values[1], values[2], values[3]),
        kS,
        kG,
        kV,
        kA);

    motorConnectedAlert.set(!inputs.motorConnected);
  }

  public Command runPrepare(double position) {
    return runOnce(() -> setGoalRotations(position));
  }

  public Command run(double position) {
    return runPrepare(position).andThen(Commands.waitUntil(this::atGoal));
  }

  public void setGoalRotations(double position) {
    io.runPosition(position, feedforward.calculate(position, 0));
    goalRotations = position;
  }

  /** Whether wrist is within tolerance of setpoint */
  public boolean atGoal() {
    return MathUtil.isNear(inputs.positionRotations, goalRotations, WristConstants.TOLERANCE);
  }

  /** Get position in rotations */
  public double getMeasuredPosition() {
    return inputs.positionRotations;
  }

  /** Get setpoint in rotations */
  public double getSetpoint() {
    return goalRotations;
  }

  public void setPid(double kP, double kI, double kD) {
    io.setPID(kP, kI, kD);
  }
}
