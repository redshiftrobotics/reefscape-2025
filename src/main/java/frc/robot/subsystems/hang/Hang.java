package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  private static final LoggedTunableNumberFactory driveFeedbackFactory =
      new LoggedTunableNumberFactory("Hang/Feedback");

  private static final LoggedTunableNumber kP =
      driveFeedbackFactory.getNumber("DriveKp", HangConstants.FEEDBACK.kP());
  private static final LoggedTunableNumber kD =
      driveFeedbackFactory.getNumber("DriveKd", HangConstants.FEEDBACK.kD());

  private HangVisualization measuredVisualizer =
      new HangVisualization("Hang/Mechanism2d/Measured", Color.kYellow);
  private HangVisualization setpointVisualizer =
      new HangVisualization("Hang/Mechanism2d/Goal", Color.kGreen);

  public Hang(HangIO io) {
    this.io = io;

    io.setPID(kP.get(), 0.0, kD.get());
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.periodic();

    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> io.setPID(values[0], 0.0, values[1]), kP, kD);

    measuredVisualizer.update(inputs.positionRotations);
  }

  private void setGoal(double goalRotations) {
    io.runPosition(goalRotations);
    setpointVisualizer.update(goalRotations);
  }

  public Command stow() {
    return runOnce(() -> setGoal(HangConstants.STOWED_POSITION_ROTATIONS));
  }

  public Command deploy() {
    return runOnce(() -> setGoal(HangConstants.DEPLOY_POSITION_ROTATIONS));
  }

  public Command retract() {
    return runOnce(() -> setGoal(HangConstants.STOWED_POSITION_ROTATIONS));
  }

  public Command coast() {
    return startEnd(
        () -> {
          io.stop();
          io.setBrakeMode(false);
        },
        () -> io.setBrakeMode(true));
  }

  /** Stop the arm from moving. */
  public void stop() {
    io.stop();
  }
}
