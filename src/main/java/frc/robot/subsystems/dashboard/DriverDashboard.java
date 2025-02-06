package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriverDashboard extends SubsystemBase {

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private DriverDashboard() {
    SmartDashboard.putString("RobotName", Constants.getRobot().toString());
    SmartDashboard.putString("RobotRoboRioSerialNumber", RobotController.getSerialNumber());
  }

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Fields ---

  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;
  private BooleanSupplier fieldRelativeSupplier;
  private BooleanSupplier hasVisionEstimate;

  // --- Setters ---

  public void addSubsystem(SubsystemBase subsystem) {
    if (subsystem instanceof Drive) {
      SmartDashboard.putData(subsystem);
    } else {
      throw new IllegalArgumentException("Unknown subsystem can not be added to driver dashboard");
    }
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

  public void setRobotSupplier(Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.speedsSupplier = robotSpeedsSupplier;
  }

  public void setFieldRelativeSupplier(BooleanSupplier fieldRelativeSupplier) {
    this.fieldRelativeSupplier = fieldRelativeSupplier;
  }

  public void setHasVisionEstimate(BooleanSupplier hasVisionEstimate) {
    this.hasVisionEstimate = hasVisionEstimate;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Game Time", Timer.getMatchTime());

    if (poseSupplier != null) {
      Pose2d pose = poseSupplier.get();
      SmartDashboard.putNumber("Heading Degrees", -pose.getRotation().getDegrees());
    }

    if (speedsSupplier != null) {
      ChassisSpeeds speeds = speedsSupplier.get();

      SmartDashboard.putNumber(
          "Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);
    }

    if (fieldRelativeSupplier != null) {
      SmartDashboard.putBoolean("Field Relative", fieldRelativeSupplier.getAsBoolean());
    }

    if (hasVisionEstimate != null) {
      SmartDashboard.putBoolean("Vision", hasVisionEstimate.getAsBoolean());
    }
  }
}
