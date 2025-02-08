package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class WristIOSparkMax implements WristIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public WristIOSparkMax() {
    // Create motor and get associated objects
    motor = new SparkMax(WristConstants.WRIST_CONFIG.motorId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    // Configure motor
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(WristConstants.WRIST_CONFIG.motorInverted()).idleMode(IdleMode.kBrake);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configurePID(double p, double i, double d) {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.closedLoop.pidf(p, i, d, 0);
    motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void moveTo(double pos, double feedforward) {
    pid.setReference(
        Units.radiansToRotations(pos) * WristConstants.GEAR_RATIO,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition() / WristConstants.GEAR_RATIO);
  }
}
