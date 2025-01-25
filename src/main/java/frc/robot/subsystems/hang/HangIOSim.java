package frc.robot.subsystems.hang;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HangIOSim implements HangIO {
  private PWMSparkMax motor = new PWMSparkMax(0);
  private Encoder encoder = new Encoder(0, 1);
  private EncoderSim encoderSim = new EncoderSim(encoder);

  // TODO: Get values from design
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(null, 0, 0, Units.inchesToMeters(18), 0, 0, false, 0, null);
  private Mechanism2d mech2d = new Mechanism2d(60, 60);
  private MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Simulated Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  private PIDController controller = new PIDController(0, 0, 0);

  @Override
  public void updateInputs(HangIOInputs inputs) {
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    armSim.update(0.02);

    encoderSim.setDistance(armSim.getAngleRads());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // arm
  }

  @Override
  public void setSetpoint(Rotation2d setpoint) {
    //
  }

  @Override
  public double getSetpoint() {
    return 0;
  }

  @Override
  public double getPosition() {
    return 0;
  }
}
