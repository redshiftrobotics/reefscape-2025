package frc.robot.subsystems.hang;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HangIOSim implements HangIO {
  // TODO
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(null, 0, 0, Units.inchesToMeters(18), 0, 0, false, 0, null);

  @Override
  public void updateInputs(HangIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}
