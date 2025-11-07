// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RotationConstants;
import frc.robot.subsystems.RotationSubsystem;
import frc.sim.Constants.RotationSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class RotationModel implements AutoCloseable {

  private final RotationSubsystem rotationSubsystem;
  private double simCurrent = 0.0;
  private SparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor rotationGearbox = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          rotationGearbox,
          RotationSimConstants.MOTOR_MOI_KG_METERS2,
          RotationConstants.MOTOR_GEAR_RATIO);

  private final DCMotorSim rotationSim = new DCMotorSim(plant, rotationGearbox);

  /** Create a new RotationModel. */
  public RotationModel(RotationSubsystem rotationSubsystemToSimulate) {

    rotationSubsystem = rotationSubsystemToSimulate;
    simulationInit();
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax and methods to set values
    sparkSim = new SparkMaxSim(rotationSubsystem.getMotor(), rotationGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "input" (voltage)
    double inputVoltage = rotationSubsystem.getVoltageCommand();
    // Apply a deadband to the input voltage to simulate static friction.
    rotationSim.setInput(
        MathUtil.applyDeadband(inputVoltage, RotationSimConstants.SIMULATED_KS, 12.0));

    // Next, we update it. The standard loop time is 20ms.
    rotationSim.update(0.020);

    // Finally, we  run the spark simulations, set our simulated encoder's readings and save the
    // current so it can be retrieved later.
    sparkSim.iterate(rotationSim.getAngularVelocityRPM(), 12.0, 0.02);
    simCurrent =
        rotationGearbox.getCurrent(rotationSim.getAngularVelocityRadPerSec(), inputVoltage);
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simCurrent;
  }

  @Override
  public void close() {
    // Nothing to close
  }
}
