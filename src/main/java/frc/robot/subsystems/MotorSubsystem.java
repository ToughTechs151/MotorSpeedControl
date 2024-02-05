// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotPreferences;

/** Motor with PID speed control. */
public class MotorSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the motor subsystem. */
  public static class Hardware {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public Hardware(CANSparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private PIDController motorController =
      new PIDController(MotorConstants.MOTOR_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          MotorConstants.MOTOR_KS_VOLTS.getValue(),
          MotorConstants.MOTOR_KV_VOLTS_PER_RPM.getValue(),
          MotorConstants.MOTOR_KA_VOLTS_PER_RPM2.getValue());

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean motorEnabled;
  private double motorVoltageCommand = 0.0;

  /** Create a new motorSubsystem controlled by a Profiled PID COntroller . */
  public MotorSubsystem(Hardware motorHardware) {
    this.motor = motorHardware.motor;
    this.encoder = motorHardware.encoder;

    initializeMotor();
  }

  private void initializeMotor() {

    RobotPreferences.initPreferencesArray(MotorConstants.getMotorPreferences());

    initMotorEncoder();
    initMotorMotor();

    // Set tolerances that will be used to determine when the motor is at the goal velocity.
    motorController.setTolerance(MotorConstants.MOTOR_TOLERANCE_RPM);

    disableMotor();
  }

  private void initMotorMotor() {
    motor.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    motor.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    motor.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Motor firmware version:" + motor.getFirmwareString());
  }

  private void initMotorEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // motor position will only be correct if the motor is in the starting rest position when
    // the subsystem is constructed.
    encoder.setPositionConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
    encoder.setVelocityConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
  }

  /**
   * Create hardware devices for the motor subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax motorMotor = new CANSparkMax(MotorConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder motorEncoder = motorMotor.getEncoder();

    return new Hardware(motorMotor, motorEncoder);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("motor Enabled", motorEnabled);
    SmartDashboard.putNumber("motor Setpoint", motorController.getSetpoint());
    SmartDashboard.putNumber("motor Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("motor Voltage", motorVoltageCommand);
    SmartDashboard.putNumber("motor Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("motor Feedforward", newFeedforward);
    SmartDashboard.putNumber("motor PID output", pidOutput);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateMotorController() {
    if (motorEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the motor at the desired speed. Store the individual values for logging.
      pidOutput = motorController.calculate(getMotorSpeed());
      newFeedforward = feedforward.calculate(motorController.getSetpoint());
      motorVoltageCommand = pidOutput + newFeedforward;

    } else {
      // If the motor isn't enabled, set the motor command to 0. In this state the motor
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      newFeedforward = 0;
      motorVoltageCommand = 0;
    }
    motor.setVoltage(motorVoltageCommand);
  }

  /** Returns a Command that runs the motor at the defined speed. */
  public Command runMotor(double setpoint) {
    return new FunctionalCommand(
        () -> setMotorSetPoint(setpoint),
        this::updateMotorController,
        interrupted -> disableMotor(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the motor. The PIDController drives the motor to this speed and holds it
   * there.
   */
  private void setMotorSetPoint(double setpoint) {
    motorController.setSetpoint(setpoint);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableMotor();
  }

  /** Returns whether the motor has reached the set point speed within limits. */
  public boolean motorAtSetpoint() {
    return motorController.atSetpoint();
  }

  /**
   * Sets up the PID controller to run the motor at the defined setpoint speed. Preferences for
   * tuning the controller are applied.
   */
  private void enableMotor() {

    // Don't enable if already enabled since this may cause control transients
    if (!motorEnabled) {
      loadPreferences();

      // Reset the PID controller to clear any previous state
      motorController.reset();
      motorEnabled = true;

      DataLogManager.log(
          "motor Enabled - kP="
              + motorController.getP()
              + " kI="
              + motorController.getI()
              + " kD="
              + motorController.getD()
              + " Setpoint="
              + motorController.getSetpoint()
              + " CurSpeed="
              + getMotorSpeed());
    }
  }

  /**
   * Disables the PID control of the motor. Sets motor output to zero. NOTE: In this state the motor
   * will slow down until it stops. Motor EMF braking will cause it to slow down faster if that mode
   * is used.
   */
  public void disableMotor() {

    // Clear the enabled flag and update the controller to zero the motor command
    motorEnabled = false;
    updateMotorController();

    // Cancel any command that is active
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log("motor Disabled CurSpeed=" + getMotorSpeed());
  }

  /** Returns the motor speed for PID control and logging (Units are RPM). */
  public double getMotorSpeed() {
    return encoder.getVelocity();
  }

  /** Returns the motor motor commanded voltage. */
  public double getMotorVoltageCommand() {
    return motorVoltageCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    motorController.setP(MotorConstants.MOTOR_KP.getValue());

    // Read Preferences for Feedforward and create a new instance
    double staticGain = MotorConstants.MOTOR_KS_VOLTS.getValue();
    double velocityGain = MotorConstants.MOTOR_KV_VOLTS_PER_RPM.getValue();
    double accelerationGain = MotorConstants.MOTOR_KA_VOLTS_PER_RPM2.getValue();
    feedforward = new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
