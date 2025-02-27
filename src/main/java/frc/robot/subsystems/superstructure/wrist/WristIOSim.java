package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Simulation of the Wrist.
 *
 * @author Aceius E.
 * @see
 *     https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/Constants.java
 * @see https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started
 */
public class WristIOSim implements WristIO {
  private final DCMotor wristGearbox = DCMotor.getNeo550(1);
  private final SparkMax sparkMax = new SparkMax(MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(sparkMax, wristGearbox);
  private final PIDController pidController = new PIDController(0, 0, 0);

  // The wrist is basically a single jointed arm, so:
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          wristGearbox,
          SIM_GEARING,
          SIM_MOMENT_OF_INERTIA,
          SIM_ARM_LENGTH,
          SIM_ARM_MIN_ANGLE,
          SIM_ARM_MAX_ANGLE,
          true,
          SIM_ARM_INIT_ANGLE);

  private double setpoint;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    // Set voltage input into simulation
    wristSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Update time
    wristSim.update(Constants.LOOP_PERIOD_SECONDS);

    // Update motor
    motorSim.iterate(
        pidController.calculate(getPosition(), setpoint),
        RoboRioSim.getVInVoltage(),
        Constants.LOOP_PERIOD_SECONDS);

    // Pull power from Sim battery
    double draw = wristSim.getCurrentDrawAmps();
    double batteryVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(draw);

    RoboRioSim.setVInVoltage(batteryVolts);

    inputs.positionRotations = getPosition();
    inputs.setpointRotations = pidController.getSetpoint();
  }

  @Override
  public void runPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  private double getPosition() {
    return Units.radiansToRotations(wristSim.getAngleRads());
  }

  @Override
  public void setPid(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }
}
