// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MotorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();

  // The driver's controller
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  // Create the motor subsystem.
  private final MotorSubsystem motor = new MotorSubsystem(MotorSubsystem.initializeHardware());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings and Shuffleboard
    configureButtonBindings();

    SmartDashboard.putData(motor);

    // Set the default motor command to control via the joystick
    motor.setDefaultCommand(
        motor.getJoystickCommand(operatorController).withName("Joystick Control"));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Run the motor at the defined speed while the right or left trigger is held.
    operatorController.rightTrigger().whileTrue(motor.runForward().withName("Motor: Run Forward"));

    operatorController.leftTrigger().whileTrue(motor.runReverse().withName("Motor: Run Reverse"));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    motor.disableMotor();
    DataLogManager.log("disableSubsystems");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the Motor Subsystem.
   *
   * @return a reference to the Motor Subsystem
   */
  public MotorSubsystem getMotorSubsystem() {
    return motor;
  }
}
