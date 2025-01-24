package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private static final DCMotor motor = DCMotor.getNEO(1);

  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  private ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              motor,
              ElevatorConstants.ELEVATOR_MASS,
              ElevatorConstants.ELEVATOR_RADIUS,
              ElevatorConstants.GEAR_RATIO),
          motor,
          ElevatorConstants.MIN_HEIGHT,
          ElevatorConstants.MAX_HEIGHT,
          true,
          ElevatorConstants.DEFAULT_HEIGHT);

  public void setGoalHeight(double targetHeight) {}

  public void updateMotors() {}

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

  public void stop() {}
}
