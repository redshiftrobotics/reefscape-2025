package frc.robot.subsystems.hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class HangIOSim implements HangIO {
  private PWMSparkMax motor = new PWMSparkMax(0);
  private Encoder encoder = new Encoder(0, 1);
  private EncoderSim encoderSim = new EncoderSim(encoder);

  // TODO: Get values from design
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(DCMotor.getNEO(1), 1, 1, Units.inchesToMeters(18), 0, 3, false, 0);
  private HangVisualization currentArm =
      new HangVisualization("Hang/Arm", 32, 32, 16, getPosition(), Color.kGreen);
  private HangVisualization setpointArm =
      new HangVisualization("Hang/Setpoint", 32, 32, 16, getPosition(), Color.kOrange);

  private PIDController controller =
      new PIDController(
          HangConstants.SIM_HANG_ARM_P, HangConstants.SIM_HANG_ARM_I, HangConstants.SIM_HANG_ARM_D);

  private double setpoint;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    motor.set(controller.calculate(getPosition(), setpoint));

    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    armSim.update(Constants.LOOP_PERIOD_SECONDS);

    encoderSim.setDistance(armSim.getAngleRads());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    currentArm.setAngle(Rotation2d.fromRadians(armSim.getAngleRads()));

    inputs.armSetpoint = setpoint;
  }

  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    setpointArm.setAngle(Rotation2d.fromRotations(setpoint));
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public double getPosition() {
    return encoderSim.getDistance();
  }

  @Override
  public void stop() {
    setpoint = getPosition();
  }
}
