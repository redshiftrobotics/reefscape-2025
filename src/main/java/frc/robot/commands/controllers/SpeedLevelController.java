package frc.robot.commands.controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayDeque;
import java.util.Deque;
import org.littletonrobotics.junction.AutoLogOutput;

/** Manages speed levels with a stack-based system, where the most recent state is active. */
public class SpeedLevelController {
  private final Deque<SpeedLevel> speedLevels;
  private final SpeedLevel defaultSpeedLevel;

  /** Creates a new SpeedController with the default speed level. */
  public SpeedLevelController() {
    this(SpeedLevel.DEFAULT);
  }

  /** Creates a new SpeedController with a specified default speed level. */
  public SpeedLevelController(SpeedLevel defaultLevel) {
    this.speedLevels = new ArrayDeque<>();
    this.defaultSpeedLevel = defaultLevel;
  }

  /** Returns the current speed level. */
  @AutoLogOutput(key = "Drive/SpeedController/speedLevel")
  public SpeedLevel getCurrentSpeedLevel() {
    return speedLevels.isEmpty() ? this.defaultSpeedLevel : speedLevels.peek();
  }

  /** Adds a speed level, making it the current active level. */
  public void pushSpeedLevel(SpeedLevel level) {
    speedLevels.push(level);
  }

  /** Removes the specified speed level if it exists in the stack. */
  public void removeSpeedLevel(SpeedLevel level) {
    speedLevels.remove(level);
  }

  /** Applies this speed level to the given chassis speeds. */
  public ChassisSpeeds apply(ChassisSpeeds speeds) {
    return apply(speeds, getCurrentSpeedLevel());
  }

  public static ChassisSpeeds apply(ChassisSpeeds speeds, SpeedLevel speedLevel) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * speedLevel.translationCoefficient,
        speeds.vyMetersPerSecond * speedLevel.translationCoefficient,
        speeds.omegaRadiansPerSecond * speedLevel.rotationCoefficient);
  }

  /** Enum representing different speed levels with translational and rotational coefficients. */
  public enum SpeedLevel {
    PRECISE(0.25, 0.1),
    DEFAULT(0.75, 0.60),
    BOOST(1.0, 0.75),
    NO_LEVEL(1.0, 1.0);

    private final double translationCoefficient;
    private final double rotationCoefficient;

    SpeedLevel(double translationCoefficient, double rotationCoefficient) {
      this.translationCoefficient = translationCoefficient;
      this.rotationCoefficient = rotationCoefficient;
    }

    /** Returns the translational coefficient for this speed level. */
    public double getTranslationCoefficient() {
      return translationCoefficient;
    }

    /** Returns the rotational coefficient for this speed level. */
    public double getRotationCoefficient() {
      return rotationCoefficient;
    }
  }
}
