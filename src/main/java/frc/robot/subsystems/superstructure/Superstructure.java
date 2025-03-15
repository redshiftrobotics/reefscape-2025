package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.utility.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends VirtualSubsystem {

  private final Elevator elevator;
  private final Wrist coralWrist;
  private final Intake coralIntake;

  public static enum State {
    STOW_LOW(0.054886473109919, -80),
    STOW_HIGH(0, 75),
    STOW_HIGHER(0, 80),

    INTAKE(0.712 - Units.inchesToMeters(4) + 0.178, -78 + 10),

    L1(0.29, -82),
    L2(0, 55),
    L3(0.478, 55),
    L4(1.445 - Units.inchesToMeters(3), 36);

    // L2_ALGEA(0.29, 0),
    // L3_ALGEA(0.478, 0);

    private static final double elevatorHeightDamageOffset = 0;
    private static final Rotation2d wristAngleDamageOffset = Rotation2d.kZero;

    private final double baseHeight;
    private final Rotation2d baseAngle;

    private double offsetHeight = 0;
    private Rotation2d offsetAngle = Rotation2d.kZero;

    private State(double height, double angleDegrees) {
      this.baseHeight = height;
      this.baseAngle = Rotation2d.fromDegrees(angleDegrees);
    }

    public void adjustHeight(double offset) {
      offsetHeight += offset;
    }

    public void adjustAngle(Rotation2d offset) {
      offsetAngle = offsetAngle.plus(offset);
    }

    public void adjustReset() {
      offsetHeight = 0;
      offsetAngle = Rotation2d.kZero;
    }

    public boolean isLevel() {
      return this == L1 || this == L2 || this == L3 || this == L4;
    }

    public boolean isIntake() {
      return this == INTAKE;
    }

    public boolean isStow() {
      return this == STOW_LOW || this == STOW_HIGH;
    }

    public double getHeight() {
      return baseHeight + offsetHeight + elevatorHeightDamageOffset;
    }

    public Rotation2d getAngle() {
      return baseAngle.plus(offsetAngle).plus(wristAngleDamageOffset);
    }
  }

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured", Color.kYellow);
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint", Color.kAliceBlue);
  private final SuperstructureVisualizer goalVisualizer =
      new SuperstructureVisualizer("Goal", Color.kLime);

  public Superstructure(Elevator elevator, Wrist coralWrist, Intake coralIntake) {
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.coralIntake = coralIntake;
  }

  public Command run(State goal) {
    return runPrepare(goal)
        .andThen(Commands.idle(elevator, coralWrist))
        .finallyDo(this::setPositionStow);
  }

  public Command run2(State goal) {
    return Commands.parallel(
            elevator.runPositionPrepare(goal::getHeight),
            coralWrist.runPositionPrepare(State.STOW_HIGH.getAngle()))
        .andThen(Commands.waitUntil(elevator::atGoalHeightRough))
        .andThen(coralWrist.runPositionPrepare(goal.getAngle()))
        .andThen(Commands.idle(elevator, coralWrist))
        .finallyDo(this::setPositionStow);
  }

  public Command runAction(State newGoal) {
    return runPrepare(newGoal).andThen(Commands.idle(elevator, coralWrist)).until(this::atGoal);
  }

  public Command runPrepare(State goal) {
    return Commands.parallel(
        elevator.runPositionPrepare(goal::getHeight),
        coralWrist.runPositionPrepare(goal::getAngle));
  }

  public Command runWheels(State goal) {
    return switch (goal) {
      case STOW_LOW -> stopIntake();
      case STOW_HIGH -> stopIntake();
      case STOW_HIGHER -> stopIntake();
      case L1 -> outtakeL1();
      case L2 -> outtake();
      case L3 -> outtake();
      case L4 -> outtake();
      case INTAKE -> intake();
    };
  }

  public Command intake() {
    return coralIntake.runMotors(-0.3);
  }

  public Command passiveIntake() {
    return coralIntake.runMotors(0);
  }

  public Command outtake() {
    return coralIntake.runMotors(-1);
  }

  public Command outtakeL1() {
    return coralIntake.runMotors(0.5, 0.3);
  }

  public Command stopIntake() {
    return coralIntake.runMotors(0);
  }

  public boolean atGoal() {
    return elevator.atGoalHeight() && coralWrist.atGoal();
  }

  public void setPositionStow() {
    if (coralIntake.hasCoral().orElse(true)) {
      elevator.setGoalHeightMeters(State.STOW_HIGH.getHeight());
      coralWrist.setGoalRotation(State.STOW_HIGH.getAngle());
    } else {
      elevator.setGoalHeightMeters(State.STOW_LOW.getHeight());
      coralWrist.setGoalRotation(State.STOW_LOW.getAngle());
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height Meters", elevator.getMeasuredHeightMeters());
    SmartDashboard.putNumber("Coral Wrist Degrees", coralWrist.getGoalRotations().getDegrees());

    for (State state : State.values()) {
      Logger.recordOutput("Superstructure/" + state.name() + "/Height", state.getHeight());
      Logger.recordOutput(
          "Superstructure/" + state.name() + "/AngleDegrees", state.getAngle().getDegrees());
      Logger.recordOutput("Superstructure/" + state.name() + "/OffsetHeight", state.offsetHeight);
      Logger.recordOutput(
          "Superstructure/" + state.name() + "/OffsetAngleDegrees", state.offsetAngle.getDegrees());
    }

    measuredVisualizer.update(
        elevator.getMeasuredHeightMeters(),
        coralWrist.getMeasuredRotation().getRadians(),
        coralIntake.getMotorsAvg());
    setpointVisualizer.update(
        elevator.getSetpoint().position,
        coralWrist.getSetpoint().position,
        coralIntake.getMotorsAvg());
    goalVisualizer.update(
        elevator.getGoalHeightMeters(),
        coralWrist.getGoalRotations().getRadians(),
        coralIntake.getMotorsAvg());
  }
}
