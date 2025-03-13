package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.GEAR_REDUCTION;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.MOTOR;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.wrist.WristConstants.WristConfig;

public class WristIOSim implements WristIO {
  private final SingleJointedArmSim arm;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean runClosedLoop = false;
  private double feedForwardVolts = 0.0;

  public WristIOSim(WristConfig config) {
    arm =
        new SingleJointedArmSim(
            MOTOR,
            GEAR_REDUCTION,
            0.025,
            Units.inchesToMeters(18),
            Units.degreesToRadians(WristConstants.MIN_POSITION_DEGREES),
            Units.degreesToRadians(WristConstants.MAX_POSITION_DEGREES),
            true,
            Units.degreesToRadians(100));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    if (runClosedLoop) {
      appliedVolts = controller.calculate(inputs.positionRad) + feedForwardVolts;
    }

    arm.setInputVoltage(appliedVolts);
    arm.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.absPositionRad = inputs.positionRad = arm.getAngleRads();
    inputs.velocityRadPerSec = arm.getVelocityRadPerSec();

    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = arm.getCurrentDrawAmps();

    inputs.motorConnected = true;
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    controller.setSetpoint(positionRad);
    feedForwardVolts = feedforward;
    runClosedLoop = true;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
