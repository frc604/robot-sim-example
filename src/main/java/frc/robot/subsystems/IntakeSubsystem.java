package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private static final TalonFX armMotor = new TalonFX(Constants.Intake.Arm.deviceID);
  private static final TalonFXSimState armMotorSim = armMotor.getSimState();
  private static final MotionMagicVoltage armMotionMagicControl = new MotionMagicVoltage(0);

  private static final TalonFX rollerMotor = new TalonFX(Constants.Intake.Roller.deviceID);
  private static final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();

  public IntakeSubsystem() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    // CTRE uses rotations, we are using radians
    armConfig.Feedback.SensorToMechanismRatio = Constants.Intake.Arm.ratio * 2.0 * Math.PI;
    armConfig.Slot0.kV = 1.1;
    armConfig.Slot0.kA = 0.03;
    armConfig.Slot0.kP = 10.0;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    armConfig.MotionMagic.MotionMagicAcceleration = 20.0;
    armMotor.getConfigurator().apply(armConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotor.getConfigurator().apply(rollerConfig);

    SmartDashboard.putData("Arm Viz", mech);

    // This assumes the arm is always stowed when the code boots.
    armMotor.setPosition(Constants.Intake.Arm.startingAngle);
    retract();
  }

  public void deploy() {
    setPosition(Constants.Intake.Arm.deployAngle);
    rollerMotor.set(1.0);
  }

  public void retract() {
    setPosition(Constants.Intake.Arm.stowAngle);
    rollerMotor.set(0.0);
  }

  private void setPosition(double position) {
    armMotionMagicControl.Slot = 0;
    armMotionMagicControl.Position = position;
    armMotor.setControl(armMotionMagicControl);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Target Position", armMotor.getClosedLoopReference().getValue());
    SmartDashboard.putNumber("Arm Actual Position", armMotor.getPosition().getValue());
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Intake.Arm.ratio,
          Constants.Intake.Arm.simMOI,
          Constants.Intake.Arm.simCGLength,
          Constants.Intake.Arm.minAngle,
          Constants.Intake.Arm.maxAngle,
          true, // Simulate gravity
          Constants.Intake.Arm.startingAngle);

  private static final FlywheelSim rollerSim =
      new FlywheelSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Intake.Roller.ratio,
          Constants.Intake.Roller.simMOI);

  // Mechanism2d Visualization
  // See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
  private static final Mechanism2d mech = new Mechanism2d(1, 1);
  private static final double kArmPivotX = 0.7;
  private static final double kArmPivotY = 0.3;
  private static final MechanismRoot2d armPivot =
      mech.getRoot("Intake Arm Pivot", kArmPivotX, kArmPivotY);
  private static final double kIntakeLength = 0.5;
  private static final MechanismLigament2d armViz =
      armPivot.append(
          new MechanismLigament2d(
              "Intake Arm", kIntakeLength, 0.0, 5.0, new Color8Bit(Color.kBlue)));

  private static final MechanismRoot2d rollerAxle = mech.getRoot("Intake Roller Axle", 0.0, 0.0);
  private static final MechanismLigament2d rollerViz =
      rollerAxle.append(
          new MechanismLigament2d("Intake Roller", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));

  @Override
  public void simulationPeriodic() {
    // ------ Update sim based on motor output
    armSim.setInput(armMotorSim.getMotorVoltage());
    armSim.update(TimedRobot.kDefaultPeriod);

    rollerSim.setInput(rollerMotorSim.getMotorVoltage());
    rollerSim.update(TimedRobot.kDefaultPeriod);

    // ------ Update motor based on sim
    // Make sure to convert radians at the mechanism to rotations at the motor
    // Subtracting out the starting angle is necessary so the simulation can't "cheat" and use the
    // sim as an absolute encoder.
    armMotorSim.setRawRotorPosition(
        (armSim.getAngleRads() - Constants.Intake.Arm.startingAngle)
            * Constants.Intake.Arm.ratio
            * 2.0
            * Math.PI);
    armMotorSim.setRotorVelocity(
        armSim.getVelocityRadPerSec() * Constants.Intake.Arm.ratio / (2.0 * Math.PI));

    double rotationsPerSecond = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rollerMotorSim.setRotorVelocity(rotationsPerSecond);
    rollerMotorSim.addRotorPosition(rotationsPerSecond * TimedRobot.kDefaultPeriod);

    // ------ Update viz based on sim
    armViz.setAngle(Math.toDegrees(armSim.getAngleRads()));

    // Update the axle as the arm moves
    rollerAxle.setPosition(
        kArmPivotX + kIntakeLength * Math.cos(armSim.getAngleRads()),
        kArmPivotY + kIntakeLength * Math.sin(armSim.getAngleRads()));
    // Scale down the angular velocity so we can actually see what is happening
    rollerViz.setAngle(
        rollerViz.getAngle()
            + Math.toDegrees(rollerSim.getAngularVelocityRPM())
                * TimedRobot.kDefaultPeriod
                * Constants.Intake.Roller.angularVelocityScalar);
  }
  // --- END STUFF FOR SIMULATION ---
}
