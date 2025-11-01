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

  private final PIDController controller = new PIDController(kP.get(), kI.get(), kD.get());
  private boolean runClosedLoop = false;

  public Hang(HangIO io) {
    this.io = io;
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);

    Logger.recordOutput("Hang/RunClosedLoop", runClosedLoop);

    if (runClosedLoop) {
      io.runOpenLoop(controller.calculate(inputs.absPositionRotations));
    }

    motorConnectedAlert.set(!inputs.motorConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> controller.setPID(values[0], values[1], values[2]), kP, kI, kD);

    measuredVisualizer.update(Rotation2d.fromRotations(inputs.positionRotations));
  }

  public void setGoal(Rotation2d rotation) {
    runClosedLoop = true;
    controller.setSetpoint(rotation.getRotations());
    setpointVisualizer.update(rotation);
  }

  public Rotation2d getMeasured() {
    return Rotation2d.fromRotations(inputs.absPositionRotations);
  }

  public Rotation2d getGoal() {
    return Rotation2d.fromRotations(controller.getSetpoint());
  }

  @AutoLogOutput(key = "Hang/MeasuredDegrees")
  public double getMeasuredDegrees() {
    return getMeasured().getDegrees();
  }

  @AutoLogOutput(key = "Hang/GoalDegrees")
  public double getGoalDegrees() {
    return getGoal().getDegrees();
  }

  @AutoLogOutput(key = "Hang/RelativeRotations")
  public double getRelativeRotations() {
    return inputs.positionRotations;
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
