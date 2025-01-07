package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Class for switching on and off features, implements BooleanSupplier */
public class OverrideSwitch implements BooleanSupplier {

  private boolean value;

  public static enum Mode {
    TOGGLE,
    HOLD
  }

  /**
   * Creates new Override switch
   *
   * @param trigger trigger to toggle state
   * @param mode either toggle or hold
   * @param defaultState default state
   */
  public OverrideSwitch(Trigger trigger, Mode mode, boolean defaultState) {
    switch (mode) {
      case TOGGLE:
        trigger.onTrue(Commands.runOnce(this::toggle));
        break;

      case HOLD:
        trigger.whileTrue(
            defaultState
                ? Commands.startEnd(this::toggleOff, this::toggleOn)
                : Commands.startEnd(this::toggleOn, this::toggleOff));
        break;
    }

    set(defaultState);
  }

  private void set(boolean value) {
    this.value = value;
  }

  public void toggle() {
    set(!value);
  }

  public void toggleOn() {
    set(true);
  }

  public void toggleOff() {
    set(false);
  }

  public boolean get() {
    return value;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
