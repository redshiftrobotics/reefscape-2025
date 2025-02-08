package frc.robot.subsystems.hang;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HangIOSim implements HangIO {
  private PWMSparkMax motor = new PWMSparkMax(0);
  private Encoder encoder = new Encoder(0, 1);
  private EncoderSim encoderSim = new EncoderSim(encoder);

  // TODO: Get values from design
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(DCMotor.getNEO(1), 1, 1, Units.inchesToMeters(18), 0, 3, false, 0);
  private LoggedMechanism2d mech2d = new LoggedMechanism2d(60, 60);
  private LoggedMechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private LoggedMechanismLigament2d arm =
      armPivot.append(
          new LoggedMechanismLigament2d(
              "Simulated Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  private PIDController controller = new PIDController(100, 0, 0); // TODO Tune

  private double setpoint;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    var x = controller.calculate(getPosition(), setpoint);
    // System.out.println(x + " set: " + setpoint + " pos: " + getPosition());
    motor.set(x);

    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    armSim.update(Constants.LOOP_PERIOD_SECONDS);

    var a = armSim.getAngleRads();
    encoderSim.setDistance(a);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    inputs.armSetpoint = setpoint;
    Logger.recordOutput("Hang/Arm", mech2d);
  }

  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
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
