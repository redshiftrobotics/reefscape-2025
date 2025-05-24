# SAASquatch Robot Code for Reefscape

This project contains robot code for 8032's robot for FRC 2025 game Reefscape.
  
## Software Features

#### Drive Train
* Custom swerve drive code for our modules, with kinematics and odometry
* All on-controller feedback loops to control precise speed and module angle
* Path following and pathfinding using PathplannerLib, can follow paths created in GUI
* High-frequency odometry (Inspired by team 6328)
* Full simulation using real swerve drive and module logic allowing for assurance of drive code and pathing logic

#### Superstructure 
* State based control to automatically reach setpoints while avoiding collisions
* Detection for when Coral is held, triggering different states
* All on-controller feedback loops, enhanced with feed forward to counter gravity and friction
* All logic fully simulated, including Coral holding allowing for easy debugging and auto creation

#### Vision
* PhotonVision based full field localization, robot always new where it was
* Smart camera placement to view relevant April Tags
* Custom advanced filtering and standard deviations calculations to throw out bad results and trust good results more
* Kalman filter to integrate vision results with odometry data
* Allowed for smart auto alignment system and consistent autos when localization was used in pathing 

#### Driving
* Two controller scheme, drive and operator
* Strategic controller rumble for drivers to understand robot  
* Elastic dashboard for key information
* Dashboard alerts for disconnected devices and other errors
* One button auto align for driver, automatically finds nearest scoring location

#### Code structure
* AdvantageKit IO layers to separate control logic with hardware implantations, allowed for logic to be simulated on fake simulated hardware
* Full logging and real time monitoring on AdvantageScope to view robot state, whether simulated or real
* WPILib style command based programming with subsystems and command for clean reusable code

![image](https://github.com/user-attachments/assets/2ff990a1-d31f-4188-9fbc-afdf1ed5c5f0)
![image](https://github.com/user-attachments/assets/36cf5fe0-066d-4685-ba1a-818f49612820)


## Robot Abilities
* 4 MK4i Swerve Modules using NEO Brushless Motor for swerve drive
* Pigion2 gyroscope for heading data, allowing for field relative drive
* Controlled by two Xbox controllers with intuitive controls 
* Two stage elevator with pivoting wrist able to reach all 4 scoring levels
* End effector with beam break sensor able to automatically obtain and release Coral
* Two OrangePi 5s with two low distortion, global shuttle Arducam OV9281s, allowing for localization and smart auto alignment
* Reliable deep hang using unique pivoting design 

## Structure
* Robot
  * Main code for robot.
  * Commands and Subsystems are folders for commands and subsystems for the Robot.
  * Utility stores utilities for the robot, such as code to flip auto directions if on the red or blue side.

## Dependencies
* WPILib
* AdvantageKit
* Pathplanner
* PhotonVision
* REVLib
