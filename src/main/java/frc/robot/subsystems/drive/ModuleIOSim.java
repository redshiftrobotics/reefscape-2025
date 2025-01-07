package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors. The flywheel sims are not physically
 * accurate, but provide a decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {

  // --- Sim Hardware ---
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  // Volts
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  // PID
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  public ModuleIOSim(ModuleConfig config) {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DriveConstants.driveMotor, 0.025, DriveConstants.driveReduction),
            DriveConstants.driveMotor);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DriveConstants.turnMotor, 0.004, DriveConstants.turnReduction),
            DriveConstants.turnMotor);

    // Create PID
    driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
    turnFeedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.LOOP_PERIOD_SECONDS);
    turnSim.update(Constants.LOOP_PERIOD_SECONDS);

    // --- Drive ---
    inputs.driveMotorConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // --- Turn ---
    inputs.turnMotorConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // --- Absolute Encoder ---
    inputs.turnAbsoluteEncoderConnected = true;
    inputs.turnAbsolutePosition = inputs.turnPosition;

    // --- Odometry ---
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedForward) {
    setDriveVoltage(
        driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadsPerSec)
            + feedForward);
  }

  @Override
  public void setTurnPosition(double angleRads) {
    setTurnVoltage(turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}

  @Override
  public void stop() {
    setDriveVoltage(0);
    setTurnVoltage(0);
  }
}
