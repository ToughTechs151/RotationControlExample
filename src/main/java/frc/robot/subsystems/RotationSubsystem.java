// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RotationConstants;
import frc.robot.util.TunableNumber;

/**
 * The {@code RotationSubsystem} class is a subsystem that controls the movement of an rotation
 * using a Profiled PID Controller. It uses a SparkMax motor and a RelativeEncoder to measure the
 * rotation's position. The class provides methods to move the rotation to a specific position, hold
 * the rotation at the current position, and shift the rotation's position up or down by a fixed
 * increment.
 *
 * <p>The RotationSubsystem class provides a constructor where hardware dependiencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of RotationSubsystem using specified hardware
 * SparkMax motor = new SparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * rotationHardware = new RotationSubsystem.Hardware(motor, encoder);
 * RotationSubsystem rotationSubsystem = new RotationSubsystem(rotationHardware);
 *
 * // Create a new instance of RotationSubsystem using default hardware
 * RotationSubsystem rotationSubsystem = new RotationSubsystem(initializeHardware());
 *
 * // Move the rotation to a specific position
 * Command moveToPositionCommand = rotationSubsystem.moveToPosition(1.0);
 * moveToPositionCommand.schedule();
 *
 * // Hold the rotation at the current position
 * Command holdPositionCommand = rotationSubsystem.holdPosition();
 * holdPositionCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the movement of an rotation using a Profiled PID Controller
 *   - Move the rotation to a specific position
 *   - Hold the rotation at the current position
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the rotation's state.
 *   - {@code useOutput()}: Generates the motor command using the PID controller and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the rotation to a new
 *     position.
 *   - {@code holdPosition()}: Returns a Command that holds the rotation at the last goal position.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the rotation has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the rotation.
 *   - {@code disable()}: Disables the PID control of the rotation.
 *   - {@code getMeasurement()}: Returns the rotation position for PID control and logging.
 *   - {@code getVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadTunableNumbers()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final SparkMax motor}: The motor used to control the rotation.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the rotation's
 *     position.
 *   - {@code private ProfiledPIDController rotationController}: The PID controller used to
 *     control the rotation's movement.
 *   - {@code private rotationFeedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double output}: The output of the PID controller.
 *   - {@code private TrapezoidProfile.State setpoint}: The setpoint of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean rotationEnabled}: A flag indicating whether the rotation is enabled.
 *   - {@code private double voltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class RotationSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the rotation subsystem. */
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

  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          RotationConstants.ROTATION_KP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              RotationConstants.ROTATION_MAX_VELOCITY_DEG_PER_SEC,
              RotationConstants.ROTATION_MAX_ACCELERATION_DEG_PER_SEC2));

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          RotationConstants.ROTATION_KS, RotationConstants.ROTATION_KV_VOLTS_PER_DEG_PER_SEC);
  // Acceleration is not used in this implementation

  private double output = 0.0;
  private TrapezoidProfile.State setpoint = new State();
  private double newFeedforward = 0;
  private boolean rotationEnabled;
  private double voltageCommand = 0.0;

  // Setup tunable numbers for the rotation.
  private TunableNumber kp = new TunableNumber("RotationKP", RotationConstants.ROTATION_KP);
  private TunableNumber ks = new TunableNumber("RotationKS", RotationConstants.ROTATION_KS);
  private TunableNumber kv =
      new TunableNumber("RotationKV", RotationConstants.ROTATION_KV_VOLTS_PER_DEG_PER_SEC);
  private TunableNumber maxVelocity =
      new TunableNumber("RotationMaxVelocity", RotationConstants.ROTATION_MAX_VELOCITY_DEG_PER_SEC);
  private TunableNumber maxAcceleration =
      new TunableNumber(
          "RotationMaxAcceleration", RotationConstants.ROTATION_MAX_ACCELERATION_DEG_PER_SEC2);

  /** Create a new RotationSubsystem controlled by a Profiled PID COntroller . */
  public RotationSubsystem(Hardware rotationHardware) {
    this.motor = rotationHardware.motor;
    this.encoder = rotationHardware.encoder;

    initializeRotation();
  }

  private void initializeRotation() {

    initMotor();

    // Set tolerances that will be used to determine when the rotation is at the goal position.
    rotationController.setTolerance(
        RotationConstants.POSITION_TOLERANCE_DEGREES, RotationConstants.VELOCITY_TOLERANCE_RPM);

    disable();

    // Add buttons to toggle brake mode
    SmartDashboard.putData(
        "Rotation Brake Mode",
        new InstantCommand(() -> setBrakeMode(true))
            .ignoringDisable(true)
            .withName("Rotation Brake"));
    SmartDashboard.putData(
        "Rotation Coast Mode",
        new InstantCommand(() -> setBrakeMode(false))
            .ignoringDisable(true)
            .withName("Rotation Coast"));
  }

  private void initMotor() {
    motorConfig.smartCurrentLimit(RotationConstants.CURRENT_LIMIT);

    // Setup the encoder scale factors. Since this is a relative encoder,
    // rotation position will only be correct if it is in the down position when
    // the subsystem is constructed.
    motorConfig.encoder.positionConversionFactor(
        RotationConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
    motorConfig.encoder.velocityConversionFactor(
        RotationConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(0);

    motor.clearFaults();
    // Configure the motor to use EMF braking when idle.
    setBrakeMode(true);

    motor.clearFaults();
    DataLogManager.log("Rotation motor firmware version:" + motor.getFirmwareString());
  }

  /**
   * Initialize hardware devices for the rotation subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax motor = new SparkMax(RotationConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    return new Hardware(motor, encoder);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Rotation Enabled", rotationEnabled);
    SmartDashboard.putNumber("Rotation Goal", rotationController.getGoal().position);
    SmartDashboard.putNumber("Rotation Position", getMeasurement());
    SmartDashboard.putNumber("Rotation Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Rotation Voltage", voltageCommand);
    SmartDashboard.putNumber("Rotation Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Rotation Feedforward", newFeedforward);
    SmartDashboard.putNumber("Rotation PID output", output);
    SmartDashboard.putNumber("Rotation SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Rotation SetPt Vel", setpoint.velocity);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void useOutput() {
    if (rotationEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      output = rotationController.calculate(getMeasurement());
      setpoint = rotationController.getSetpoint();

      // Calculate the feedforward to move the rotation at the desired velocity and offset
      // the effect of gravity. Voltage for acceleration is not used.
      newFeedforward = feedforward.calculate(setpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;

    } else {
      // If the rotation isn't enabled, set the motor command to 0. In this state the rotation
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      output = 0;
      newFeedforward = 0;
      voltageCommand = 0;
    }
    motor.setVoltage(voltageCommand);
  }

  /** Returns a Command that moves the rotation to a new position. */
  public Command moveToPosition(double goal) {
    return new FunctionalCommand(
        () -> setGoalPosition(goal),
        this::useOutput,
        interrupted -> {},
        this::atGoalPosition,
        this);
  }

  /**
   * Returns a Command that holds the rotation at the last goal position using the PID Controller
   * driving the motor.
   */
  public Command holdPosition() {
    return run(this::useOutput).withName("Rotation: Hold Position");
  }

  /**
   * Set the goal state for the subsystem, limited to allowable range. Goal velocity is set to zero.
   * The ProfiledPIDController drives the rotation to this position and holds it there.
   */
  private void setGoalPosition(double goal) {
    rotationController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.RotationConstants.ROTATION_MIN_POSITION_DEGREES,
                Constants.RotationConstants.ROTATION_MAX_POSITION_DEGREES),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the rotation has reached the goal position and velocity is within limits. */
  public boolean atGoalPosition() {
    return rotationController.atGoal();
  }

  /**
   * Sets up the PID controller to move the rotation to the defined goal position and hold at that
   * position. Preferences for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!rotationEnabled) {
      loadTunableNumbers();
      setDefaultCommand(holdPosition());

      // Reset the PID controller to clear any previous state
      rotationController.reset(getMeasurement());
      rotationEnabled = true;

      DataLogManager.log(
          "Rotation Enabled - kP="
              + rotationController.getP()
              + " kI="
              + rotationController.getI()
              + " kD="
              + rotationController.getD()
              + " PosGoal="
              + rotationController.getGoal().position
              + " CurPos="
              + getMeasurement());
    }
  }

  /**
   * Disables the PID control of the rotation. Sets motor output to zero. NOTE: In this state the
   * rotation will move until it hits the stop. Using EMF braking mode with motor will slow this
   * movement.
   */
  public void disable() {

    // Clear the enabled flag and call useOutput to zero the motor command
    rotationEnabled = false;
    useOutput();

    // Remove the default command and cancel any command that is active
    removeDefaultCommand();
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log(
        "Rotation Disabled CurPos=" + getMeasurement() + " CurVel=" + encoder.getVelocity());
  }

  /**
   * Returns the rotation position for PID control and logging (Units are meters from low position).
   */
  public double getMeasurement() {
    // Add the offset from the starting point. The rotation must be at this position at startup for
    // the relative encoder to provide a correct position.
    return encoder.getPosition() + RotationConstants.ROTATION_OFFSET_DEGREES;
  }

  /** Returns the Motor Commanded Voltage. */
  public double getVoltageCommand() {
    return voltageCommand;
  }

  /** Returns the motor for simulation. */
  public SparkMax getMotor() {
    return motor;
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    if (enableBrake) {
      DataLogManager.log("Rotation motor set to brake mode");
      brakeConfig.idleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Rotation motor set to coast mode");
      brakeConfig.idleMode(IdleMode.kCoast);
    }
    motor.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Load values that can be tuned at runtime. This should only be called when the controller is
   * disabled - for example from enable().
   */
  private void loadTunableNumbers() {

    // Read values for PID controller
    rotationController.setP(kp.get());

    // Read values for Trapezoid Profile and update
    rotationController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Read values for Feedforward and create a new instance
    feedforward = new SimpleMotorFeedforward(ks.get(), kv.get());
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
