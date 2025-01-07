package frc.robot.subsystems.examples.flywheel;

import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.FLYWHEEL_CONFIG;
import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.GEAR_RATIO;

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

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "SparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private final SparkMax leader;
  private final SparkMax follower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public FlywheelIOSparkMax() {

    // --- Save config ---
    leader = new SparkMax(FLYWHEEL_CONFIG.followerID(), MotorType.kBrushless);
    follower = new SparkMax(FLYWHEEL_CONFIG.leaderID(), MotorType.kBrushless);

    // --- Set up leader controller ---
    encoder = leader.getEncoder();
    pid = leader.getClosedLoopController();

    // --- Configure Hardware ---

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .inverted(FLYWHEEL_CONFIG.leaderInverted())
        .idleMode(IdleMode.kCoast);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kCoast)
        .follow(leader, FLYWHEEL_CONFIG.followerInverted());

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts =
        new double[] {
          leader.getAppliedOutput() * leader.getBusVoltage(),
          follower.getAppliedOutput() * follower.getBusVoltage()
        };
    inputs.supplyCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(kP, kI, kD, 0.0);
    leader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
