package frc.robot.robots;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LoggedRobot;

public class RobotTypeChecker extends LoggedRobot {
  @Override
  public void robotInit() {
    if (RobotBase.isReal()) {
      SmartDashboard.putString("Robot Type", "Real");
    } else if (RobotBase.isSimulation()) {
      SmartDashboard.putString("Robot Type", "Simulation");
    } else {
      SmartDashboard.putString("Robot Type", "Unknown");
    }

    SmartDashboard.putNumber("Team Number", RobotController.getTeamNumber());
    SmartDashboard.putString("Serial Number", RobotController.getSerialNumber());
    System.out.println("Serial Number: " + RobotController.getSerialNumber());

    SmartDashboard.putNumber("Brownout Voltage", RobotController.getBrownoutVoltage());
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }
}
