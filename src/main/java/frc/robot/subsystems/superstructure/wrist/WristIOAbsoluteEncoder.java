package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.TOLERANCE;

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

  // TODO: This class has a lot of issues.

  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final CANcoder encoder;
  private final Supplier<Angle> positionSupplier;

  private double setpoint;

  public WristIOAbsoluteEncoder(int motorId, int encoderId) {
    SparkMaxConfig config = new SparkMaxConfig();

    motor = new SparkMax(motorId, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: This is a problem, the PID controller is using the relative encoder on the spark by
    // default,
    // you need to sync the spark motor controller with the CANCoder position, or use a Rio based
    // PID controller and just read the CANCoder position
    // check how it is used in ModuleIOSparkMax.java line 160, you can use the same method here
    pidController = motor.getClosedLoopController();

    // TODO: if you choose to use the spark max pid controller, you need to set the gear ratio (as
    // the coefficient)
    // Again, to do this check ModuleIOSparkMax.java line 103-104

    encoder = new CANcoder(encoderId);
    // TODO: this encoder needs configuring, it's "zero" position is not necessarily our zero
    // position, it is random
    // Although it is constants, we still want the zero to be something reasonable, especially if we
    // are using it to set the wrist to a certain rotations
    // Check how it is done in ModuleIOSparkMax.java line 85-90

    positionSupplier = encoder.getPosition().asSupplier();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.setpointRotations = setpoint;
    // TODO: forgot to update position
  }

  @Override
  public void runPosition(double setpoint) {
    this.setpoint = setpoint;

    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setPid(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);

    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
