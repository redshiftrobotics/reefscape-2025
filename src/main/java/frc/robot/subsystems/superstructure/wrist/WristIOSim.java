package frc.robot.subsystems.superstructure.wrist;

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
            config.gearReduction(),
            0.025,
            Units.inchesToMeters(18),
            0,
            Math.PI * 2,
            false,
            0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    if (runClosedLoop) {
      appliedVolts = controller.calculate(inputs.positionRotations) + feedForwardVolts;
    }

    arm.setInputVoltage(appliedVolts);
    arm.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionRotations = Units.radiansToRotations(arm.getAngleRads());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(arm.getVelocityRadPerSec());

    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.supplyCurrentAmps = new double[] {arm.getCurrentDrawAmps()};
  }

  @Override
  public void runPosition(double positionRotations) {
    controller.setSetpoint(positionRotations);
    feedForwardVolts = 0;
    runClosedLoop = true;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
