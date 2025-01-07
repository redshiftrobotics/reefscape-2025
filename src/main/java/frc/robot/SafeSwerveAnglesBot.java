package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;

public class SafeSwerveAnglesBot extends LoggedRobot {

  private final Map<String, CANcoder> cancoderMap = new HashMap<>();
  private final List<SparkMax> driveMotors = new ArrayList<>();

  private final CANcoder frontLeftCancoder =
      new CANcoder(DriveConstants.FRONT_LEFT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder frontRightCancoder =
      new CANcoder(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder backLeftCancoder =
      new CANcoder(DriveConstants.BACK_LEFT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder backRightCancoder =
      new CANcoder(DriveConstants.BACK_RIGHT_MODULE_CONFIG.absoluteEncoderChannel());

  public SafeSwerveAnglesBot() {
    cancoderMap.put("frontLeft", frontLeftCancoder);
    cancoderMap.put("frontRight", frontRightCancoder);
    cancoderMap.put("backLeft", backLeftCancoder);
    cancoderMap.put("backRight", backRightCancoder);

    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    magnetSensorConfig.AbsoluteSensorDiscontinuityPoint = 1;
    magnetSensorConfig.MagnetOffset = 0;

    for (CANcoder cancoder : cancoderMap.values()) {
      cancoder.getConfigurator().apply(magnetSensorConfig);
    }

    driveMotors.add(
        new SparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless));
    driveMotors.add(
        new SparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless));
    driveMotors.add(
        new SparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless));
    driveMotors.add(
        new SparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless));
  }

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Drive Speed", 0);
    SmartDashboard.putBoolean("Print", false);
  }

  @Override
  public void robotPeriodic() {
    for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
      SmartDashboard.putNumber(
          entry.getKey() + " Position",
          entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
    }

    if (SmartDashboard.getBoolean("Print", false)) {
      for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
        System.out.println(
            entry.getKey()
                + " Position: "
                + entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    double speed = SmartDashboard.getNumber("Drive Speed", 0);
    for (SparkMax sparkMax : driveMotors) {
      sparkMax.set(speed);
    }
  }

  @Override
  public void teleopExit() {
    for (SparkMax sparkMax : driveMotors) {
      sparkMax.set(0);
    }
  }
}
