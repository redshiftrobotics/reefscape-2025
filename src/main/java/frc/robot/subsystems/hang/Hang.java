package frc.robot.subsystems.hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  private static final LoggedTunableNumberFactory hangFactory =
      new LoggedTunableNumberFactory("Hang");

  private static final LoggedTunableNumber kP =
      hangFactory.getNumber("kP", HangConstants.FEEDBACK.kP());
  private static final LoggedTunableNumber kI =
      hangFactory.getNumber("kI", HangConstants.FEEDBACK.kI());
  private static final LoggedTunableNumber kD =
      hangFactory.getNumber("kD", HangConstants.FEEDBACK.kD());

  private final HangVisualization measuredVisualizer =
      new HangVisualization("Measured", Color.kYellow);
  private final HangVisualization setpointVisualizer = new HangVisualization("Goal", Color.kGreen);

  private final Alert motorConnectedAlert = new Alert("Hang Motor Disconnected", AlertType.kError);

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean runClosedLoop = false;

  public Hang(HangIO io) {
    this.io = io;
    io.setBrakeMode(true);
    io.setLimits(HangConstants.RELATIVE_MIN_ROTATIONS, HangConstants.RELATIVE_MAX_ROTATIONS);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);

    Logger.recordOutput("Hang/RunClosedLoop", runClosedLoop);

    if (runClosedLoop) {
      io.runOpenLoop(controller.calculate(inputs.positionRotations));
    }

    motorConnectedAlert.set(!inputs.motorConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> controller.setPID(values[0], values[1], values[2]), kP, kI, kD);

    measuredVisualizer.update(Rotation2d.fromRotations(inputs.positionRotations));
  }

  private void setGoal(double rotations) {
    runClosedLoop = true;
    controller.setSetpoint(rotations);
    setpointVisualizer.update(Rotation2d.fromRotations(rotations));
  }

  @AutoLogOutput(key = "Hang/MeasuredPositionRotations")
  public double getMeasuredPosition() {
    return inputs.positionRotations;
  }

  @AutoLogOutput(key = "Hang/LastGoalPositionRotations")
  public double getGoal() {
    return controller.getSetpoint();
  }

  public Command stow() {
    return runOnce(() -> setGoal(HangConstants.STOWED_POSITION_ROTATIONS));
  }

  public Command deploy() {
    return runOnce(() -> setGoal(HangConstants.DEPLOY_POSITION_ROTATIONS));
  }

  public Command retract() {
    return runOnce(() -> setGoal(HangConstants.RETRACT_POSITION_ROTATIONS));
  }

  public Command runSet(double speed) {
    return runEnd(() -> io.runOpenLoop(speed), io::stop);
  }

  public void set(double speed) {
    runClosedLoop = false;
    io.runOpenLoop(speed);
  }

  /** Stop the arm from moving. */
  public void stop() {
    runClosedLoop = false;
    io.stop();
  }
}
