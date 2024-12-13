// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.MotorSubsystem;
import frc.sim.Constants.MotorSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class MotorModel implements AutoCloseable {

  private final MotorSubsystem motorSubsystem;
  private double simMotorCurrent = 0.0;
  private SparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor motorGearbox = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          motorGearbox, MotorSimConstants.MOTOR_MOI_KG_METERS2, MotorConstants.MOTOR_GEAR_RATIO);

  private final DCMotorSim motorSim = new DCMotorSim(plant, motorGearbox);

  /** Create a new MotorModel. */
  public MotorModel(MotorSubsystem motorSubsystemToSimulate) {

    motorSubsystem = motorSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new SparkMaxSim(motorSubsystem.getMotor(), motorGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {

    double inputVoltage = motorSubsystem.getVoltageCommand();
    motorSim.setInput(inputVoltage);

    // Next, we update it. The standard loop time is 20ms.
    motorSim.update(0.020);

    double simMotorSpeed = motorSim.getAngularVelocityRPM();

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.iterate(simMotorSpeed, 12.0, 0.02);
    simMotorCurrent = motorGearbox.getCurrent(1.0, inputVoltage);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simMotorCurrent));
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simMotorCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
