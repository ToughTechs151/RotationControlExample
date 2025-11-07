// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.RotationSubsystem;

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
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  // Now all the subsystems.
  // The Rotation.
  private final RotationSubsystem robotRotation =
      new RotationSubsystem(RotationSubsystem.initializeHardware());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Publish subsystem data including commands
    SmartDashboard.putData(robotRotation);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Move the rotation to the start position when the 'A' button is pressed.
    driverController
        .a()
        .onTrue(
            robotRotation
                .moveToPosition(Constants.RotationConstants.ROTATION_START_POSITION_DEGREES)
                .withName("Rotation: Move to Start Position"));

    // Move the rotation to the end position when the 'B' button is pressed.
    driverController
        .b()
        .onTrue(
            robotRotation
                .moveToPosition(Constants.RotationConstants.ROTATION_END_POSITION_DEGREES)
                .withName("Rotation: Move to End Position"));

    // Disable the rotation controller when the 'X' button is pressed.
    // NOTE: This is intended for initial rotation testing and should be removed in the final robot
    // to prevent accidental disable resulting in lowering of the rotation.
    driverController.x().onTrue(Commands.runOnce(robotRotation::disable));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    robotRotation.disable();
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
   * Use this to get the Rotation Subsystem.
   *
   * @return the command to run in autonomous
   */
  public RotationSubsystem getRotationSubsystem() {
    return robotRotation;
  }
}
