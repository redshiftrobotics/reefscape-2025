package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.TOLERANCE;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.WRIST_D;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.WRIST_FF;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.WRIST_I;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.WRIST_P;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public class WristIOAbsoluteEncoder implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final CANcoder encoder;
  private final Supplier<Angle> positionSupplier;

  private double setpoint;

  public WristIOAbsoluteEncoder(int motorId, int encoderId) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(WRIST_P, WRIST_I, WRIST_D, WRIST_FF);

    motor = new SparkMax(motorId, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = motor.getClosedLoopController();

    encoder = new CANcoder(encoderId);

    positionSupplier = encoder.getPosition().asSupplier();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.setpoint = setpoint;
  }

  @Override
  public void goTo(double setpoint) {
    this.setpoint = setpoint;

    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(setpoint, positionSupplier.get().in(Units.Rotation), TOLERANCE);
  }

  @Override
  public void setPid(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);

    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
