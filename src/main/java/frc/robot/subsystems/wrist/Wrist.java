package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs;
  private final ArmFeedforward feedforward;

  public Wrist(WristIO wIO) {
    io = wIO;
    feedforward =
        new ArmFeedforward(
            WristConstants.FEED_FORWARD_CONFIG.s(),
            WristConstants.FEED_FORWARD_CONFIG.v(),
            WristConstants.FEED_FORWARD_CONFIG.g(),
            WristConstants.FEED_FORWARD_CONFIG.a());
    io.configurePID(
        WristConstants.PID_CONFIG.p(),
        WristConstants.PID_CONFIG.i(),
        WristConstants.PID_CONFIG.d());
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  public void setRotation(Rotation2d rot) {
    io.moveTo(rot.getRadians(), feedforward.calculate(rot.getRadians(), 0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }
}
