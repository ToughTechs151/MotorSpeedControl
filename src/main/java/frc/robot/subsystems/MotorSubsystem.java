// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.TunableNumber;

/** Motor with PID plus feedforward speed control. */
public class MotorSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the motor subsystem. */
  public static class Hardware {
    SparkMax motor;
    RelativeEncoder encoder;

    public Hardware(SparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final EncoderConfig encoderConfig = new EncoderConfig();

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean motorEnabled;
  private double motorVoltageCommand = 0.0;
  private double maxSpeed = 0.0;

  // Setup tunable numbers and controllers for the motor.
  private TunableNumber proportionalGain =
      new TunableNumber("Motor Kp", MotorConstants.MOTOR_KP_VOLTS_PER_RPM);
  private TunableNumber staticGain = new TunableNumber("Motor Ks", MotorConstants.MOTOR_KS_VOLTS);
  private TunableNumber velocityGain =
      new TunableNumber("Motor Kv", MotorConstants.MOTOR_KV_VOLTS_PER_RPM);
  private TunableNumber accelerationGain =
      new TunableNumber("Motor Ka", MotorConstants.MOTOR_KA_VOLTS_PER_RPM2);
  private TunableNumber fixedRpm =
      new TunableNumber("Motor Fixed Speed", MotorConstants.MOTOR_FIXED_SPEED_RPM);
  private TunableNumber joystickRpm =
      new TunableNumber("Motor Joystick Speed", MotorConstants.MOTOR_MAX_JOYSTICK_SPEED_RPM);

  private PIDController motorController = new PIDController(proportionalGain.get(), 0.0, 0.0);

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(staticGain.get(), velocityGain.get(), accelerationGain.get());

  /** Create a new motorSubsystem controlled by a Profiled PID COntroller . */
  public MotorSubsystem(Hardware motorHardware) {
    this.motor = motorHardware.motor;
    this.encoder = motorHardware.encoder;

    initializeMotor();
  }

  private void initializeMotor() {

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(40);

    // Setup the encoder scale factors
    encoderConfig.positionConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
    encoderConfig.velocityConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
    motorConfig.apply(encoderConfig);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.clearFaults();

    DataLogManager.log("Motor firmware version:" + motor.getFirmwareString());

    // Set tolerances that will be used to determine when the motor is at the goal velocity.
    motorController.setTolerance(MotorConstants.MOTOR_TOLERANCE_RPM);

    disableMotor();
  }

  /**
   * Create hardware devices for the motor subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax motor = new SparkMax(MotorConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    return new Hardware(motor, encoder);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("motor Enabled", motorEnabled);
    SmartDashboard.putNumber("motor Setpoint", motorController.getSetpoint());
    SmartDashboard.putNumber("motor Feedforward", newFeedforward);
    SmartDashboard.putNumber("motor PID output", pidOutput);
    // duplicate velocity here since value on Shuffleboard tab may be invalid in simulation
    SmartDashboard.putNumber("motor Velocity", encoder.getVelocity());
  }

  /**
   * Generate the motor command using the PID controller output and feedforward. Save the individual
   * values for logging.
   */
  public void updateMotorController() {
    if (motorEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the motor at the desired speed. Store the individual values for logging.
      pidOutput = motorController.calculate(getSpeed());
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

  /** Returns a Command that runs the motor forward at a tunable set speed. */
  public Command runForward() {
    return new FunctionalCommand(
        () -> setFixedSetPoint(1.0),
        this::updateMotorController,
        interrupted -> disableMotor(),
        () -> false,
        this);
  }

  /** Returns a Command that runs the motor in reverse at a tunable set speed. */
  public Command runReverse() {
    return new FunctionalCommand(
        () -> setFixedSetPoint(-1.0),
        this::updateMotorController,
        interrupted -> disableMotor(),
        () -> false,
        this);
  }

  /** Setup the command to control via joystick using a tunable max speed. */
  public Command getJoystickCommand(CommandXboxController driverController) {

    return new FunctionalCommand(
        this::setMaxAndEnable,
        () -> {
          motorController.setSetpoint(
              -maxSpeed
                  * MathUtil.applyDeadband(
                      driverController.getLeftY(), MotorConstants.JOYSTICK_DEADBAND));
          updateMotorController();
        },
        interrupted -> disableMotor(),
        () -> false,
        this);
  }

  /** Set the max speed using the tunable number and enable the motor. */
  private void setMaxAndEnable() {
    maxSpeed = joystickRpm.get();

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableMotor();
  }

  /**
   * Set the setpoint for the motor based on the tunable number. The PIDController drives the motor
   * to this speed and holds it there.
   */
  private void setFixedSetPoint(double scale) {
    motorController.setSetpoint(scale * fixedRpm.get());

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
      loadPIDFTunableNumbers();

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
              + getSpeed());
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
    DataLogManager.log("motor Disabled CurSpeed=" + getSpeed());
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    if (enableBrake) {
      DataLogManager.log("Motor set to brake mode");
      motorConfig.idleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Motor set to coast mode");
      motorConfig.idleMode(IdleMode.kCoast);
    }
    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Returns the motor speed for PID control and logging (Units are RPM). */
  public double getSpeed() {
    return encoder.getVelocity();
  }

  /** Returns the motor motor commanded voltage. */
  public double getVoltageCommand() {
    return motorVoltageCommand;
  }

  /** Returns the motor current. */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Load PIDF values that can be tuned at runtime. This should only be called when the controller
   * is disabled - for example from enable().
   */
  private void loadPIDFTunableNumbers() {

    // Read tunable values for PID controller
    motorController.setP(proportionalGain.get());

    // Read tunable values for Feedforward and create a new instance
    feedforward =
        new SimpleMotorFeedforward(staticGain.get(), velocityGain.get(), accelerationGain.get());
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
