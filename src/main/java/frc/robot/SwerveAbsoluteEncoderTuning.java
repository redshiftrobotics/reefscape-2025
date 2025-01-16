package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;

public class SwerveAbsoluteEncoderTuning extends LoggedRobot {

  private final Map<String, CANcoder> cancoderMap = new HashMap<>();

  private final Timer timer = new Timer();

  private final CANcoder frontLeftCancoder =
      new CANcoder(DriveConstants.FRONT_LEFT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder frontRightCancoder =
      new CANcoder(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder backLeftCancoder =
      new CANcoder(DriveConstants.BACK_LEFT_MODULE_CONFIG.absoluteEncoderChannel());
  private final CANcoder backRightCancoder =
      new CANcoder(DriveConstants.BACK_RIGHT_MODULE_CONFIG.absoluteEncoderChannel());

  public SwerveAbsoluteEncoderTuning() {
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
  }

  @Override
  public void robotInit() {
    timer.restart();
  }

  @Override
  public void robotPeriodic() {
    for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
      SmartDashboard.putNumber(
          entry.getKey() + " Position",
          entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
    }

    if (timer.advanceIfElapsed(1)) {
      System.out.println("Swerve Positions");
      for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
        System.out.println(
            entry.getKey()
                + " Position: "
                + entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
      }
    }
  }
}
