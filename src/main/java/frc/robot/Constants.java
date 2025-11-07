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

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to enable using Tunable Numbers
  public static final boolean TUNING_MODE = true;

  /** Constants used for the Rotation subsystem. */
  public static final class RotationConstants {

    private RotationConstants() {
      throw new IllegalStateException("RotationConstants Utility Class");
    }

    // Set this to use the profiled PID controller, otherwise a regular PID controller is used.
    public static final boolean USE_PROFILED = false;

    public static final int MOTOR_PORT = 1;
    public static final int CURRENT_LIMIT = 40;

    public static final double MOTOR_GEAR_RATIO =
        12.0; // Ratio of motor rotations to output rotations after gearing and pulleys
    public static final double MOTOR_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / MOTOR_GEAR_RATIO;

    // Constants tunable through TunableNumbers
    // These are fake gains for simulation; these must be determined individually for each robot
    public static final double ROTATION_KP = 24.0 / 90.0 * MOTOR_GEAR_RATIO;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_KS = 0.0;
    public static final double ROTATION_KV_VOLTS_PER_DEG_PER_SEC = 0.12 * MOTOR_GEAR_RATIO;
    public static final double ROTATION_MAX_VELOCITY_DEG_PER_SEC = 30.0;
    public static final double ROTATION_MAX_ACCELERATION_DEG_PER_SEC2 = 120.0;

    // Encoder is reset to measure 0 at the starting position.
    public static final double ROTATION_OFFSET_DEGREES = 0.0;
    public static final double ROTATION_MIN_POSITION_DEGREES = 0.0;
    public static final double ROTATION_MAX_POSITION_DEGREES = 90.0;

    public static final double POSITION_TOLERANCE_DEGREES = 1.0;
    public static final double VELOCITY_TOLERANCE_RPM = 1.0;

    public static final double ROTATION_START_POSITION_DEGREES = 0.0;
    public static final double ROTATION_END_POSITION_DEGREES = 90.0;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
