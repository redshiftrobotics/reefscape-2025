package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.MOTOR_ID;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_ARM_INIT_ANGLE;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_ARM_LENGTH;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_ARM_MAX_ANGLE;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_ARM_MIN_ANGLE;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_GEARING;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.SIM_MOMENT_OF_INERTIA;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.TOLERANCE;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Simulation of the Wrist. gearbox The type of and number of motors in the arm gearbox.
 *
 * <p>gearing The gearing of the arm (numbers greater than 1 represent reductions).
 *
 * <p>jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
 *
 * <p>armLengthMeters The length of the arm.
 *
 * <p>minAngleRads The minimum angle that the arm is capable of.
 *
 * <p>maxAngleRads The maximum angle that the arm is capable of.
 *
 * <p>simulateGravity Whether gravity should be simulated or not.
 *
 * <p>startingAngleRads The initial position of the Arm simulation in radians.
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
  private final SparkClosedLoopController pidController = sparkMax.getClosedLoopController();

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
        Units.radiansPerSecondToRotationsPerMinute(wristSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.LOOP_PERIOD_SECONDS);

    // Pull power from Sim battery
    double draw = wristSim.getCurrentDrawAmps();
    double batteryVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(draw);

    RoboRioSim.setVInVoltage(batteryVolts);

    inputs.position = getPosition();
    inputs.setpoint = setpoint;
  }

  @Override
  public void goTo(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(setpoint, getPosition(), TOLERANCE);
  }

  private double getPosition() {
    return Units.radiansToRotations(wristSim.getAngleRads());
  }
}
