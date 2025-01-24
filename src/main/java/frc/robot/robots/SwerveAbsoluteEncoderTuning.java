package frc.robot.robots;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;

public class SwerveAbsoluteEncoderTuning extends LoggedRobot {

  private final Map<String, CANcoder> cancoderMap = new HashMap<>();

  private final Timer timer = new Timer();

  private final boolean BREAK_MODE = false;

  private final SparkMax[] turns = {
    new SparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG.turnID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.turnID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG.turnID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG.turnID(), MotorType.kBrushless),
  };

  private final SparkMax[] drives = {
    new SparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG.driveID(), MotorType.kBrushless),
    new SparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG.driveID(), MotorType.kBrushless),
  };

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

    SmartDashboard.putBoolean("Break Mode", BREAK_MODE);

    for (SparkMax sparkMax : turns) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(BREAK_MODE ? IdleMode.kBrake : IdleMode.kCoast);
      sparkMax.configure(
          config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void robotPeriodic() {
    for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
      SmartDashboard.putNumber(
          entry.getKey() + " Position",
          entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
    }

    if (timer.advanceIfElapsed(3)) {
      System.out.println();
      System.out.println("Swerve Positions");
      for (Map.Entry<String, CANcoder> entry : cancoderMap.entrySet()) {
        System.out.println(
            entry.getKey()
                + " Position: "
                + entry.getValue().getAbsolutePosition().refresh().getValueAsDouble());
      }
    }
  }

  @Override
  public void autonomousInit() {
    for (SparkMax sparkMax : drives) {
      sparkMax.set(0.1);
    }
  }

  @Override
  public void autonomousExit() {
    for (SparkMax sparkMax : drives) {
      sparkMax.stopMotor();
    }
  }
}
