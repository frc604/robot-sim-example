# Robot Simulation Example

A barebones robot simulation example that utilizes:
- CTRE swerve & simuation
- WPILib arm and flywheel physics simulation
- PhotonVision simulation
- AdvantageScope

Note that this is meant to be used as reference, not as a base template for your robot.

https://github.com/user-attachments/assets/3b000901-9fe6-4cc3-b8e9-8271f580be2a

## Key Elements
- [CTRE swerve](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java)
    - Simulation built in
- [Example intake subsystem](src/main/java/frc/robot/subsystems/IntakeSubsystem.java)
    - Utilizes WPILib's `SingleJointedArmSim` and `FlywheelSim` to simulate arm and roller physics
    - Visualized with WPILib's [Mechanism2d](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html) widget
- [PhotonVision sim](src/main/java/frc/robot/subsystems/VisionSystem.java)
    - Updated based on the CTRE swerve sim pose
    - Visualized using the built in Field2d widget & web GUI
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) for 3d viz and plotting
    - Basic layout [provided](advantagescope_layout.json)

## Additional Resources
- [WPILib simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html)
- [WPILib physics simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)
- [PhotonVision simulation](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html)
- [Mechanism2d](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html)
- [AdvantageScope 3D Field](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md)

## Controls

- Left stick X/Y: Swerve translate
- Right stick X: Swerve rotate
- Right bumper: Deploy intake when held

## Setup

### With just WPILib simulation GUI
- Ensure the Arm Viz and Sim Field are displayed

![WPILib simgui](images/wpilib-simgui.png)

### With AdvantageScope
- In AdvantageScope, click `File > Import Layout` and select `advantagescope_layout.json`.
- Connect to the local simulator `File > Connect to Simulator`

![AdvantageScope viz](images/advantagescope-viz.png)

### PhotonVision
- View PhotonVision camera sim at http://localhost:1182

![PhotonVision viz](images/photonvision-viz.png)
