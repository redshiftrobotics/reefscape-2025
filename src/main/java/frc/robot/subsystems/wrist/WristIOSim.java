package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  private static final DCMotor motor = DCMotor.getNeo550(1);
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motor,
          WristConstants.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(WristConstants.ARM_LENGTH, WristConstants.ARM_MASS),
          WristConstants.ARM_LENGTH,
          Units.degreesToRadians(-45),
          Units.degreesToRadians(45),
          true,
          0);
  private PIDController pid =
      new PIDController(
          WristConstants.PID_CONFIG.p(),
          WristConstants.PID_CONFIG.i(),
          WristConstants.PID_CONFIG.d());

  private double feedforwardVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void configurePID(double p, double i, double d) {
    pid.setPID(p, i, d);
  }

  @Override
  public void moveTo(double pos, double feedforward) {
    pid.setSetpoint(pos);
    feedforwardVolts = feedforward;
  }

  @Override
  public void stop() {
    sim.setInputVoltage(0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(pid.calculate(sim.getAngleRads()) + feedforwardVolts, -12.0f, 12.0f);
    sim.setInputVoltage(appliedVolts);

    sim.update(0.02);

    inputs.positionRad = sim.getAngleRads();
    inputs.setpointRad = pid.getSetpoint();
    inputs.appliedVolts = appliedVolts;
    inputs.angularVelocity = sim.getVelocityRadPerSec();
  }
}
