package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.elevator.ElevatorConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final elevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SparkMax motor;

  /** Creates a new Flywheel. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void goToStage(int stage) {
    switch ()
  }

  /** Reset. */
  public void reset() {
    io.goToHeight(0);
    io.stop();
  }
}