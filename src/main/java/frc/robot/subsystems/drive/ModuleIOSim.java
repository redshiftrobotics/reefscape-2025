package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

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

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFFVolts = 0;

  public ModuleIOSim() {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ModuleConstants.DRIVE_MOTOR, 0.025, ModuleConstants.DRIVE_REDUCTION),
            ModuleConstants.DRIVE_MOTOR);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ModuleConstants.TURN_MOTOR, 0.004, ModuleConstants.TURN_REDUCTION),
            ModuleConstants.TURN_MOTOR);

    // Create PID
    driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);
    turnFeedback = new PIDController(0.0, 0.0, 0.0, Constants.LOOP_PERIOD_SECONDS);

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Run closed-loop control

    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveFeedback.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnFeedback.calculate(turnSim.getAngularPositionRad());
    } else {
      turnFeedback.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));

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
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = volts;
  }

  @Override
  public void setDriveVelocity(double velocityRadsPerSec, double feedForward) {
    driveClosedLoop = true;
    driveFFVolts = feedForward;
    driveFeedback.setSetpoint(velocityRadsPerSec);
  }

  @Override
  public void setTurnPosition(double angleRads) {
    turnClosedLoop = true;
    turnFeedback.setSetpoint(angleRads);
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
