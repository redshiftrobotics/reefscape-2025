package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
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

  private HangVisualization measuredVisualizer =
      new HangVisualization("Hang/Mechanism2d/Measured", Color.kYellow);
  private HangVisualization setpointVisualizer =
      new HangVisualization("Hang/Mechanism2d/Goal", Color.kGreen);

  public Hang(HangIO io) {
    this.io = io;

    io.setPID(kP.get(), kI.get(), kD.get());
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);

    SmartDashboard.putNumber("Hang Rotations", inputs.positionRotations);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> io.setPID(values[0], values[1], values[2]), kP, kI, kD);

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

  public Command runSet(double speed) {
    return runEnd(() -> io.runOpenLoop(speed), io::stop);
  }

  public void set(double speed) {
    io.runOpenLoop(speed);
  }

  /** Stop the arm from moving. */
  public void stop() {
    io.stop();
  }
}
