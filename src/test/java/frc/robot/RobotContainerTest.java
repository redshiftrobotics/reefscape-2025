package frc.robot;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Tests for RobotContainer */
public class RobotContainerTest {

  @Test
  @DisplayName("Instantiate RobotContainer")
  public void createRobotContainer() {
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate RobotContainer, see stack trace above.");
    }
  }
}
