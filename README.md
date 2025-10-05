# SAASquatch Robot Code for Reefscape

This project contains robot code for 8032's robot for FRC 2025 game Reefscape.

![Simulation Demo Video](https://github.com/user-attachments/assets/82d81fd8-b114-4d88-bd92-67431d91ece3)

## Software Features

#### Drive Train
* Custom swerve drive code for our modules, with kinematics and odometry
* All on-controller feedback loops to control precise speed and module angle
* Path following and pathfinding using PathplannerLib, can follow paths created in GUI
* High-frequency odometry (Inspired by team 6328)
* Full simulation using real swerve drive and module logic allowing for assurance of drive code and pathing logic

<!-- Simulated drive train, pathfollowing AND back and forth swerve bot -->

#### Driving
* Two controller scheme, drive and operator
* Strategic controller rumble for drivers to understand robot
* Elastic dashboard for key information
* Dashboard alerts for disconnected devices and other errors
* One button auto align for driver, automatically finds nearest scoring location
<!-- real robot scoring AND sim auto alignment  -->

<img width="350" alt="Driver Xbox Control Scheme" src="https://github.com/user-attachments/assets/80fe2d9a-7706-4655-ab8e-83e323117ebf" />
<img width="350" alt="Operator Xbox Control Scheme" src="https://github.com/user-attachments/assets/bddfd905-e608-4e04-a1af-e1560d8cafb6" />


#### Superstructure
* State based control to automatically reach setpoints while avoiding collisions
* Detection for when Coral is held, triggering different states
* All on-controller feedback loops, enhanced with feed forward to counter gravity and friction
* All logic fully simulated, including Coral holding allowing for easy debugging and auto creation
<!-- Real robot scoring on practice field AND simulated view of robot with operator control -->

#### Vision
* PhotonVision based full field localization
* Smart camera placement to view  April Tags
* Custom advanced filtering and standard deviations calculations to throw out bad results and trust good results more
* Kalman filter to integrate vision results with odometry data
* Allowed for smart auto alignment system and consistent autos when localization was used in pathing
<!-- Simulated vision AND real vision video AND vision setup diagram -->

#### Code structure
* AdvantageKit IO layers to separate control logic with hardware implantations, allowed for logic to be simulated on fake simulated hardware
* Full logging and real time monitoring on AdvantageScope to view robot state, whether simulated or real
* WPILib style command based programming with subsystems and command for clean reusable code

<details>
<summary>Diagram of code structure</summary>
  
Subsystem design with AKit IO layers.
![IO Layer Diagram](https://github.com/user-attachments/assets/2ff990a1-d31f-4188-9fbc-afdf1ed5c5f0)

Overall command based design.
![Command Based Diagram](https://github.com/user-attachments/assets/36cf5fe0-066d-4685-ba1a-818f49612820)

</details>


## Robot Abilities
* 4 MK4i Swerve Modules using NEO Brushless Motor for swerve drive
* Pigion2 gyroscope for heading data, allowing for field relative drive
* Controlled by two Xbox controllers with intuitive controls
* Two stage elevator with pivoting wrist able to reach all 4 scoring levels
* End effector with beam break sensor able to automatically obtain and release Coral
* Two OrangePi 5s with two low distortion, global shuttle Arducam OV9281s, allowing for localization and smart auto alignment
* Reliable deep hang using unique pivoting design

## Dependencies
* WPILib
* AdvantageKit
* Pathplanner
* PhotonVision
* REVLib
