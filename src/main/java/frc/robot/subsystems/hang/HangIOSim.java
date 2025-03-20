package frc.robot.subsystems.hang;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class HangIOSim implements HangIO {

  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          MOTOR,
          HangConstants.GEAR_REDUCTION,
          0.0025,
          Units.inchesToMeters(18),
          -Math.PI,
          +Math.PI,
          false,
          0);

  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean runClosedLoop = false;
  private double feedForwardVolts = 0.0;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.motorConnected = true;

    if (runClosedLoop) {
      appliedVolts = controller.calculate(inputs.positionRotations) + feedForwardVolts;
    }

    if (DriverStation.isDisabled()) {
      appliedVolts = 0;
    }

    arm.setInputVoltage(appliedVolts);

    arm.update(Constants.LOOP_PERIOD_SECONDS);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(arm.getCurrentDrawAmps()));

    inputs.positionRotations =
        inputs.absPositionRotations = Units.radiansToRotations(arm.getAngleRads());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(arm.getVelocityRadPerSec());

    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.supplyCurrentAmps = new double[] {arm.getCurrentDrawAmps()};
  }

  @Override
  public void runOpenLoop(double output) {
    runVolts(output * 12.0);
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    runClosedLoop = false;
  }

  @Override
  public void stop() {
    runOpenLoop(0);
  }
}
