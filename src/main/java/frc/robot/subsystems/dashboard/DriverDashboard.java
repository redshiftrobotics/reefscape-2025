package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriverDashboard extends SubsystemBase {

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private final Field2d field = new Field2d();

  private DriverDashboard() {
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putString("RobotName", Constants.getRobot().toString());
    SmartDashboard.putString("RobotRoboRioSerialNumber", RobotController.getSerialNumber());
  }

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Fields ---

  private BooleanSupplier fieldRelativeSupplier;
  private BooleanSupplier headingControlledSupplier;

  private Supplier<Pose2d> poseSupplier;
  private Supplier<Pose2d> autoAlginPoseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;

  private BooleanSupplier hasVisionEstimate;
  private Debouncer debouncer;

  private BooleanSupplier usingIntakeSensor;
  private BooleanSupplier hasCoral;

  private BooleanSupplier superstructureAtGoal;

  private DoubleSupplier hangValue;

  // --- Setters ---

  public void addSubsystem(SubsystemBase subsystem) {
    SmartDashboard.putData(subsystem);
  }

  public void addCommand(String name, Runnable runnable, boolean runsWhenDisabled) {
    addCommand(name, Commands.runOnce(runnable), runsWhenDisabled);
  }

  public void addCommand(String name, Command command, boolean runsWhenDisabled) {
    SmartDashboard.putData(name, command.withName(name).ignoringDisable(runsWhenDisabled));
  }

  public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.poseSupplier = robotPoseSupplier;
  }

  public void setAutoAlignPoseSupplier(Supplier<Pose2d> autoAlignPoseSupplier) {
    this.autoAlginPoseSupplier = autoAlignPoseSupplier;
  }

  public void setRobotSupplier(Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.speedsSupplier = robotSpeedsSupplier;
  }

  public void setFieldRelativeSupplier(BooleanSupplier fieldRelativeSupplier) {
    this.fieldRelativeSupplier = fieldRelativeSupplier;
  }

  public void setHeadingControlledSupplier(BooleanSupplier headingControlledSupplier) {
    this.headingControlledSupplier = headingControlledSupplier;
  }

  public void setHasVisionEstimateSupplier(BooleanSupplier hasVisionEstimate, double debounceTime) {
    this.hasVisionEstimate = hasVisionEstimate;
    debouncer = new Debouncer(debounceTime, DebounceType.kFalling);
  }

  public void setSensorSuppliers(BooleanSupplier usingIntakeSensor, BooleanSupplier hasCoral) {
    this.usingIntakeSensor = usingIntakeSensor;
    this.hasCoral = hasCoral;
  }

  public void setHangSuppliers(DoubleSupplier hangValue) {
    this.hangValue = hangValue;
  }

  public void setSuperstructureAtGoal(BooleanSupplier superstructureAtGoal) {
    this.superstructureAtGoal = superstructureAtGoal;
  }

  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Game Time", DriverStation.getMatchTime());

    if (poseSupplier != null) {
      Pose2d pose = poseSupplier.get();
      SmartDashboard.putNumber("Heading Degrees", ((-pose.getRotation().getDegrees() + 360) % 360));
      field.setRobotPose(pose);

      // SmartDashboard.putNumber(
      // "Distance To Reef [Tag 8] [Test]",
      // Units.metersToInches(
      //     Math.abs(
      //             AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
      //                 .getTagPose(18)
      //                 .get()
      //                 .toPose2d()
      //                 .minus(pose)
      //                 .getX())
      //         - DRIVE_CONFIG.bumperCornerToCorner().getX() / 2.0));
    }

    if (autoAlginPoseSupplier != null) {
      Pose2d pose = autoAlginPoseSupplier.get();
      field
          .getObject("Auto Align Pose")
          .setPose(pose == null ? new Pose2d(-100, -100, Rotation2d.k180deg) : pose);
    }

    if (speedsSupplier != null) {
      ChassisSpeeds speeds = speedsSupplier.get();

      SmartDashboard.putNumber(
          "Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);
    }

    if (fieldRelativeSupplier != null) {
      SmartDashboard.putBoolean("Field Relative", fieldRelativeSupplier.getAsBoolean());
    }

    if (headingControlledSupplier != null) {
      SmartDashboard.putBoolean("Heading Controlled", headingControlledSupplier.getAsBoolean());
    }

    if (hasVisionEstimate != null) {
      SmartDashboard.putBoolean(
          "Has Vision", debouncer.calculate(hasVisionEstimate.getAsBoolean()));
    }

    if (usingIntakeSensor != null) {
      SmartDashboard.putBoolean("Intake Sensor?", usingIntakeSensor.getAsBoolean());
    }

    if (hasCoral != null) {
      SmartDashboard.putBoolean("Has Coral?", hasCoral.getAsBoolean());
    }

    if (superstructureAtGoal != null) {
      SmartDashboard.putBoolean("At Goal?", superstructureAtGoal.getAsBoolean());
    }

    if (hangValue != null) {
      SmartDashboard.putNumber("Hang Value", hangValue.getAsDouble());
    }
  }
}
