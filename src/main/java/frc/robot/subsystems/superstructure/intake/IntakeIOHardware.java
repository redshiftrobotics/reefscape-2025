package frc.robot.subsystems.superstructure.intake;

import static frc.robot.utility.SparkUtil.ifOkMulti;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.superstructure.intake.IntakeConstants.IntakeConfig;
import frc.robot.utility.SparkUtil;
import java.util.function.DoubleSupplier;

/** Hardware implementation of the TemplateIO. */
public class IntakeIOHardware implements IntakeIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  public IntakeIOHardware(IntakeConfig config) {
    leftMotor = new SparkMax(config.motorIdLeft(), MotorType.kBrushless);
    rightMotor = new SparkMax(config.motorIdRight(), MotorType.kBrushless);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig
        .smartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT)
        .inverted(config.invertedLeft())
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0);

    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig
        .smartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT)
        .inverted(config.invertedRight())
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speedLeft = leftMotor.get();
    inputs.speedRight = rightMotor.get();

    SparkUtil.clearStickyFault();

    ifOkMulti(
        new SparkMax[] {leftMotor, rightMotor},
        new DoubleSupplier[] {
          () -> leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
          () -> rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()
        },
        values -> inputs.appliedVolts = values);
    ifOkMulti(
        new SparkMax[] {leftMotor, rightMotor},
        new DoubleSupplier[] {leftMotor::getOutputCurrent, rightMotor::getOutputCurrent},
        values -> inputs.supplyCurrentAmps = values);

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
  }

  @Override
  public void setLeftMotor(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  @Override
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
