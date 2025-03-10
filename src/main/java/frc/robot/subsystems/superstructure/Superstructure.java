package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.utility.VirtualSubsystem;

public class Superstructure extends VirtualSubsystem {

  private final Elevator elevator;
  private final Wrist coralWrist;
  private final Intake coralIntake;

  public static enum State {
    STOW,
    INTAKE,

    L1(false),
    L2(false),
    L3(false),
    L4(false);

    // L2_ALGAE(true),
    // L3_ALGAE(true);

    public boolean isLevel() {
      return this == L1 || this == L2 || this == L3 || this == L4;
    }

    public boolean isAlgae() {
      return algae;
    }

    public State asAlgae() {
      return switch (this) {
          // case L2 -> L2_ALGAE;
          // case L3 -> L3_ALGAE;
        default -> this;
      };
    }

    public boolean algae;

    private State() {
      this.algae = false;
    }

    private State(boolean algae) {
      this.algae = algae;
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

  public Command runPrepare(State newGoal) {
    return switch (newGoal) {
      case STOW -> stowLow();
      case L1 -> prepareL1();
      case L2 -> prepareL2();
      case L3 -> prepareL3();
      case L4 -> prepareL4();
      case INTAKE -> prepareIntake();
    };
  }

  public static final double L1_HEIGHT = 1.82;
  public static final double L1_CORAL_ANGLE = -6.71;

  public static final double L2_HEIGHT = 0.599 + Units.inchesToMeters(1);
  public static final double L2_CORAL_ANGLE = -7.690;

  public static final double L3_HEIGHT = 1.035 + Units.inchesToMeters(2);
  public static final double L3_CORAL_ANGLE = -7.643;

  public static final double L4_HEIGHT = L3_HEIGHT;
  public static final double L4_CORAL_ANGLE = L3_CORAL_ANGLE;

  public static final double INTAKE_HEIGHT = 0.218 + Units.inchesToMeters(1.5);
  public static final double INTAKE_CORAL_ANGLE = -4.214;

  public static final double STOW_HEIGHT = 0;
  public static final double STOW_CORAL_ANGLE = -1.619;

  public Command prepareL1() {
    return Commands.parallel(
        elevator.runPositionPrepare(L1_HEIGHT), coralWrist.runPositionPrepare(L1_CORAL_ANGLE));
  }

  public Command prepareL2() {
    return Commands.parallel(
        elevator.runPositionPrepare(L2_HEIGHT), coralWrist.runPositionPrepare(L2_CORAL_ANGLE));
  }

  public Command prepareL3() {
    return Commands.parallel(
        elevator.runPositionPrepare(L3_HEIGHT), coralWrist.runPositionPrepare(L3_CORAL_ANGLE));
  }

  public Command prepareL4() {
    return Commands.parallel(
        elevator.runPositionPrepare(L4_HEIGHT), coralWrist.runPositionPrepare(L4_CORAL_ANGLE));
  }

  public Command prepareIntake() {
    return Commands.parallel(
        elevator.runPositionPrepare(INTAKE_HEIGHT),
        coralWrist.runPositionPrepare(INTAKE_CORAL_ANGLE));
  }

  public Command stowLow() {
    return Commands.parallel(
        elevator.runPositionPrepare(STOW_HEIGHT), coralWrist.runPositionPrepare(STOW_CORAL_ANGLE));
  }

  public Command intake() {
    return coralIntake.runMotors(-0.6);
  }

  public Command passiveIntake() {
    return coralIntake.runMotors(-0.05);
  }

  public Command outtake() {
    return coralIntake.runMotors(1);
  }

  public Command stopIntake() {
    return coralIntake.runMotors(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", elevator.getHeightMeters());
    SmartDashboard.putNumber("Coral Wrist Position", coralWrist.getMeasuredPosition());

    measuredVisualizer.update(
        elevator.getHeightMeters(),
        coralWrist.getMeasuredPosition(),
        coralIntake.isIntakeRunning());
    setpointVisualizer.update(
        elevator.getSetpoint().position, coralWrist.getGoal(), coralIntake.isIntakeRunning());
    goalVisualizer.update(
        elevator.getGoalHeightMeters(), coralWrist.getGoal(), coralIntake.isIntakeRunning());
  }
}
