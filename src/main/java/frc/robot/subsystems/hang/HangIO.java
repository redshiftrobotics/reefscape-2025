package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    //
  }

  public void updateInputs(HangIOInputs inputs);
}
