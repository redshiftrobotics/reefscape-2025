package frc.robot.subsystems.examples.flywheel;

import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.FLYWHEEL_CONFIG;
import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.GEAR_RATIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;

  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;

  public FlywheelIOTalonFX() {

    leader = new TalonFX(FLYWHEEL_CONFIG.leaderID());
    follower = new TalonFX(FLYWHEEL_CONFIG.followerID());

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderCurrent = leader.getSupplyCurrent();

    followerCurrent = follower.getSupplyCurrent();

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        FLYWHEEL_CONFIG.leaderInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.Clockwise_Positive;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(
        new Follower(
            leader.getDeviceID(),
            FLYWHEEL_CONFIG.followerInverted() ^ FLYWHEEL_CONFIG.leaderInverted()));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = new double[] {leaderAppliedVolts.getValueAsDouble()};
    inputs.supplyCurrentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(
        new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)).withFeedForward(ffVolts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double Kp, double Ki, double Kd) {
    var config = new Slot0Configs();
    config.kP = Kp;
    config.kI = Ki;
    config.kD = Kd;
    leader.getConfigurator().apply(config);
  }
}
