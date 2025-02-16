package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Hardware implementation of the TemplateIO. */
public class WristIORelativeEncoder implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;

  private double setpoint;

  public WristIORelativeEncoder(int motorId) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(WRIST_P, WRIST_I, WRIST_D, WRIST_FF);

    motor = new SparkMax(motorId, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.setpoint = setpoint;

    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void goTo(double setpoint) {
    this.setpoint = setpoint;
  }
}
