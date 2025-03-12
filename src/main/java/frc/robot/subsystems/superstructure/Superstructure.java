package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SetAddressableLEDPattern;
import frc.robot.subsystems.addressableled.AddressableLEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.utility.VirtualSubsystem;

public class Superstructure extends VirtualSubsystem {

  private final Elevator elevator;
  private final Wrist coralWrist;
  private final Intake coralIntake;
  private final AddressableLEDSubsystem lights;

  public static enum State {
    STOW,
    STOW_HIGH,
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

  public Superstructure(
      Elevator elevator, Wrist coralWrist, Intake coralIntake, AddressableLEDSubsystem lights) {
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.coralIntake = coralIntake;
    this.lights = lights;
  }

  public Command run(State goal) {
    return runPrepare(goal)
        .andThen(Commands.idle(elevator, coralWrist))
        .finallyDo(
            () -> {
              elevator.setGoalHeightMeters(STOW_HEIGHT);
              coralWrist.setGoalRotation(STOW_CORAL_ANGLE);
            });
  }

  public Command runAction(State newGoal) {
    return runPrepare(newGoal)
        .andThen(Commands.idle(elevator, coralWrist))
        .until(() -> elevator.atGoalHeight() && coralWrist.atGoal());
  }

  public Command runPrepare(State goal) {
    return switch (goal) {
      case STOW -> stowLow();
      case STOW_HIGH -> stowHigh();
      case L1 -> prepareL1();
      case L2 -> prepareL2();
      case L3 -> prepareL3();
      case L4 -> prepareL4();
      case INTAKE -> prepareIntake();
    };
  }

  public static final double L1_HEIGHT = 0.151148670108898;
  public static final Rotation2d L1_CORAL_ANGLE = Rotation2d.fromDegrees(70);

  public static final double L2_HEIGHT = 0.469491383230037 + Units.inchesToMeters(3);
  public static final Rotation2d L2_CORAL_ANGLE = Rotation2d.fromDegrees(-35);

  public static final double L3_HEIGHT = 1.035 + Units.inchesToMeters(2);
  public static final Rotation2d L3_CORAL_ANGLE = Rotation2d.fromDegrees(-35);

  public static final double L4_HEIGHT = L3_HEIGHT;
  public static final Rotation2d L4_CORAL_ANGLE = L3_CORAL_ANGLE;

  public static final double INTAKE_HEIGHT = 0.218 + Units.inchesToMeters(1.5);
  public static final Rotation2d INTAKE_CORAL_ANGLE = Rotation2d.fromDegrees(35);

  public static final double STOW_HEIGHT = 0.054886473109919;
  public static final Rotation2d STOW_CORAL_ANGLE = Rotation2d.fromDegrees(-90);
  public static final Rotation2d STOW_CORAL_ANGLE_HIGH = Rotation2d.fromDegrees(90);

  // TODO: Integrate lightstrip commands into this stuff
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

  public Command stowHigh() {
    return Commands.parallel(
        elevator.runPositionPrepare(STOW_HEIGHT),
        coralWrist.runPositionPrepare(STOW_CORAL_ANGLE_HIGH));
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

  public Command outtakeL1() {
    return coralIntake.runMotors(0.5, 0.3);
  }

  public Command stopIntake() {
    return coralIntake.runMotors(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height Meters", elevator.getMeasuredHeightMeters());
    SmartDashboard.putNumber("Coral Wrist Degrees", coralWrist.getGoalRotations().getDegrees());

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
