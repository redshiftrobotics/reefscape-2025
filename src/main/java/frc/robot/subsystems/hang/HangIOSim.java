package frc.robot.subsystems.hang;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class HangIOSim implements HangIO {
  private final PWMSparkMax motor = new PWMSparkMax(0);
  private final Encoder encoder = new Encoder(0, 1);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          HangConstants.GEAR_REDUCTION,
          1,
          Units.inchesToMeters(18),
          0,
          3,
          false,
          0);

  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean runClosedLoop = false;
  private double feedForwardVolts = 0.0;

  @Override
  public void updateInputs(HangIOInputs inputs) {

    if (runClosedLoop) {
      appliedVolts = controller.calculate(inputs.positionRotations) + feedForwardVolts;
    }

    motor.set(appliedVolts);

    arm.setInputVoltage(motor.get());

    arm.update(Constants.LOOP_PERIOD_SECONDS);

    encoderSim.setDistance(arm.getAngleRads());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(arm.getCurrentDrawAmps()));

    inputs.positionRotations = Units.radiansToRotations(arm.getAngleRads());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(arm.getVelocityRadPerSec());

    inputs.appliedVolts = new double[] {motor.getVoltage()};
    inputs.supplyCurrentAmps = new double[] {arm.getCurrentDrawAmps()};
  }

  @Override
  public void runPosition(double positionRotations) {
    controller.setSetpoint(positionRotations);
    feedForwardVolts = 0;
    runClosedLoop = true;
  }

  @Override
  public void runOpenLoop(double output) {
    runVolts(MathUtil.inverseInterpolate(-12.0, 12.0, output));
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

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
