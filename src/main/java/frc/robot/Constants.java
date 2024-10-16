// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  // Enable tuning the PID and feedforward controller constants
  public static final boolean TUNING_MODE = true;

  /** Constants used for the Motor subsystem. */
  public static final class MotorConstants {

    private MotorConstants() {
      throw new IllegalStateException("MotorConstants Utility Class");
    }

    public static final int MOTOR_PORT = 9;

    // Constants tunable through TunableNumbers
    public static final double MOTOR_FIXED_SPEED_RPM = 2000.0;
    public static final double MOTOR_MAX_JOYSTICK_SPEED_RPM = 4000.0;
    public static final double MOTOR_KP_VOLTS_PER_RPM = 6.0 / 1000;
    public static final double MOTOR_KS_VOLTS = 0.0;
    public static final double MOTOR_KV_VOLTS_PER_RPM = 6.0 / 5000;
    public static final double MOTOR_KA_VOLTS_PER_RPM2 = 0.0;

    public static final double MOTOR_GEAR_RATIO =
        1.0; // Ratio of motor rotations to output rotations
    public static final double MOTOR_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / MOTOR_GEAR_RATIO;
    public static final double MOTOR_TOLERANCE_RPM = 100;
    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }
}
