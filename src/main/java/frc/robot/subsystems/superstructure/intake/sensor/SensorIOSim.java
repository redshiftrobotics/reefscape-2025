package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.wpilibj.Timer;
import java.util.Random;

public class SensorIOSim implements SensorIO {

  public static final double MAX_TIME_TILL_ITEM_SECONDS = 5.0;

  private final Timer timer = new Timer();
  private final Random random = new Random();

  private double timeTillItem = 0.0;
  private boolean itemRequested = false;

  private boolean hasItem = false;

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;

    if (itemRequested) {
      if (timer.hasElapsed(timeTillItem)) {
        hasItem = true;
      }
    } else {
      hasItem = false;
    }

    inputs.altDetected = inputs.detected = hasItem;
    inputs.rawValue = inputs.detected ? 1.0 : 0.0;
    inputs.rawVolts = inputs.detected ? 5.0 : 4.0;
  }

  @Override
  public void simulateItemDesire() {
    timer.start();
    timeTillItem = random.nextDouble() * MAX_TIME_TILL_ITEM_SECONDS;
    itemRequested = true;
  }

  @Override
  public void simulateItemEjection() {
    itemRequested = false;
    hasItem = false;
  }
}
