package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leader;
  private final SparkMax follower;
  private final RelativeEncoder encoder;
  private boolean closedLoop = false;
  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  public ElevatorIOSparkMax() {
    leader = new SparkMax(ElevatorConstants.motor1ID, MotorType.kBrushless);
    follower = new SparkMax(ElevatorConstants.motor2ID, MotorType.kBrushless);
    encoder = leader.getEncoder();

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast);
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast)
        .follow(leader, true);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoalHeight(double targetHeight) {}

  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  public void updateMotors() {
    if (closedLoop) {
      double pidVal = controller.calculate(encoder.getPosition());
      leader.setVoltage(pidVal + feedforward.calculate(controller.getSetpoint().velocity));
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
    leader.stopMotor();
  }
}
