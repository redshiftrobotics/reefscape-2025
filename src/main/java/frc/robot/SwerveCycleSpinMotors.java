package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.List;

public class SwerveCycleSpinMotors extends Robot {

  private record NamedMotor(String name, SparkMax motor) {
  }

  private final List<NamedMotor> motors = new ArrayList<>();
  private final Timer timer = new Timer();

  private final double SPEED = 0.5;
  private final double TIME_PER_MOTOR = 3.5;

  private int motorIndex = 0;

  public SwerveCycleSpinMotors() {

    // Drive Motors
    motors.add(
        new NamedMotor(
            "FrontLeftDrive",
            new SparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "FrontRightDrive",
            new SparkMax(
                DriveConstants.FRONT_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "BackLeftDrive",
            new SparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "BackRightDrive",
            new SparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless)));

    // Turn motors
    motors.add(
        new NamedMotor(
            "FrontLeftTurn",
            new SparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG.turnID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "FrontRightTurn",
            new SparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.turnID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "BackLeftTurn",
            new SparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG.turnID(), MotorType.kBrushless)));
    motors.add(
        new NamedMotor(
            "BackRightTurn",
            new SparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG.turnID(), MotorType.kBrushless)));
  }

  @Override
  public void robotInit() {
    motorIndex = 0;
    timer.restart();    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putString("Motor", String.format("%s: %s", motorIndex, motors.get(motorIndex).name()));

    if (timer.advanceIfElapsed(TIME_PER_MOTOR)) {
      motors.get(motorIndex).motor().stopMotor();
      motorIndex = (motorIndex + 1) % motors.size();
      System.out.println(String.format("Setting %s motor to %s speed", motors.get(motorIndex).name(), SPEED));
    }

    SparkMax motor = motors.get(motorIndex).motor();

    motor.set(SPEED);

    SmartDashboard.putNumber("Motor Speed (-1.0 to 1.0)", motor.get());
    SmartDashboard.putNumber("Motor Output Current (Amps)", motor.getOutputCurrent());
    SmartDashboard.putNumber("Motor Voltage (Volts)", motor.getAppliedOutput() * motor.getBusVoltage());
    SmartDashboard.putNumber("Motor Temp (Celsius)", motor.getMotorTemperature());

  }
}
