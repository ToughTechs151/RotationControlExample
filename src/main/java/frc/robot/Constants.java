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

    // Set this for a mechanism that can rotate fully with no stops. The angle wraps between
    // the minimum and maximum angle, e.g. +180 / -180 degrees
    public static final boolean CONTINUOUS = false;

    public static final int MOTOR_PORT = 1;
    public static final int CURRENT_LIMIT = 40;

    public static final double GEAR_RATIO =
        12.0; // Ratio of motor rotations to output rotations after gearing and pulleys
    public static final double OUTPUT_DEGREES_PER_ENCODER_ROTATION = 360.0 / GEAR_RATIO;
    public static final double OUTPUT_DEG_PER_SEC_PER_ENCODER_RPM =
        OUTPUT_DEGREES_PER_ENCODER_ROTATION / 60.0;

    // Constants tunable through TunableNumbers
    // These are fake gains for simulation; these must be determined individually for each robot
    public static final double ROTATION_KP = 12.0 / 180.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_KS = 0.2;
    public static final double ROTATION_KV_VOLTS_PER_DEG_PER_SEC =
        12.0 / (5000.0 / 60.0 * 360.0 / GEAR_RATIO);
    public static final double ROTATION_MAX_VELOCITY_DEG_PER_SEC = 90.0;
    public static final double ROTATION_MAX_ACCELERATION_DEG_PER_SEC2 = 180.0;

    // Encoder is reset to measure 0 at the starting position.
    public static final double ROTATION_OFFSET_DEGREES = 0.0;
    public static final double ROTATION_MIN_POSITION_DEGREES = -180.0;
    public static final double ROTATION_MAX_POSITION_DEGREES = 180.0;

    public static final double POSITION_TOLERANCE_DEGREES = 1.0;
    public static final double VELOCITY_TOLERANCE_RPM = 1.0;

    public static final double ROTATION_START_POSITION_DEGREES = 0.0;
    public static final double ROTATION_FORWARD_POSITION_DEGREES = 120.0;
    public static final double ROTATION_BACK_POSITION_DEGREES = -120.0;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
