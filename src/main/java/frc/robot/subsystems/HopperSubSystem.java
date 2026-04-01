// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubSystem extends SubsystemBase {

  // Fields
  private SparkFlex hopperMotor;

  /** Creates a new HopperSubSystem. */
  public HopperSubSystem() {
    // Intialize and Configure
    hopperMotor = new SparkFlex(Constants.FuelConstants.FUEL_HOPPER_ID, MotorType.kBrushless);

    // Set Configuration - See Example in FuelSubSystem

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Methods
  // Set Power Method

  // Set Position Method

  // Command Factory
  // Set Power Command

  // Set Position Command

}
