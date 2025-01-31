package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private boolean closedLoop = false;
  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  public ElevatorIOSparkMax() {
    motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  public void setGoalHeight(double targetHeight) {}

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void updateMotors() {
    if (closedLoop) {
      double pidVal = controller.calculate(encoder.getPosition());
      motor.setVoltage(pidVal + feedforward.calculate(controller.getSetpoint().velocity));
    }
  }

  public void configurePID(
      double maxVelocity,
      double maxAcceleration,
      double Kp,
      double Ki,
      double Kd,
      double Ks,
      double Kg,
      double Kv) {
    constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    controller = new ProfiledPIDController(Kp, Ki, Kd, constraints, Constants.LOOP_PERIOD_SECONDS);
    feedforward = new ElevatorFeedforward(Ks, Kg, Kv);
  }

  public void stop() {
    motor.stopMotor();
  }
}
