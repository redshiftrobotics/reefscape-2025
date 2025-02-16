package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Simulation implementation of the TemplateIO. */
public class ElevatorIOSim implements ElevatorIO {

  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  private final ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              MOTOR,
              ElevatorConstants.carriageMassKg,
              ElevatorConstants.drumRadius,
              ElevatorConstants.gearReduction),
          MOTOR,
          0.0,
          ElevatorConstants.carriageMaxHeight,
          true,
          0);

  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean runClosedLoop = false;
  private double feedForwardVolts = 0.0;

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (runClosedLoop) {
      appliedVolts = controller.calculate(inputs.positionRad) + feedForwardVolts;
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionRad = sim.getPositionMeters() / ElevatorConstants.drumRadius;
    inputs.velocityRadPerSec = sim.getVelocityMetersPerSecond() / ElevatorConstants.drumRadius;
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};

    inputs.appliedVolts = new double[] {appliedVolts};

    inputs.motorConnected = true;
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    controller.setSetpoint(positionRad);
    feedForwardVolts = feedforward;
    runClosedLoop = true;
  }

  @Override
  public void runOpenLoop(double output) {
    runVolts(MathUtil.inverseInterpolate(-12.0, +12.0, output));
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
    runClosedLoop = false;
  }

  @Override
  public void stop() {
    runOpenLoop(0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
