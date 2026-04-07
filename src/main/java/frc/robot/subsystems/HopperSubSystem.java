// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubSystem extends SubsystemBase {

  // Fields
  private SparkFlex hopperMotor;
  private RelativeEncoder hopperEncoder;
  private SparkClosedLoopController hopperClosedLoopController;
  private SparkFlexConfig hopperConfig;

  // Add fields for encoder, configuration, and closed Loop Controller
  // See the ClimberSubSystem for examples


  /** Creates a new HopperSubSystem. */
  public HopperSubSystem() {
    // Intialize and Configure
    hopperMotor = new SparkFlex(Constants.FuelConstants.FUEL_HOPPER_ID, MotorType.kBrushless);

    // Set Configuration - See Examples from ClimberSubSystem
    hopperEncoder = hopperMotor.getEncoder();
    hopperClosedLoopController = hopperMotor.getClosedLoopController();
    
    hopperConfig = new SparkFlexConfig();
    hopperConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    hopperConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.1).i(0).d(0).outputRange(-0.08, 0.08);

    hopperEncoder.setPosition(0);

    hopperConfig.smartCurrentLimit(Constants.FuelConstants.CURRENT_LIMIT);
    hopperConfig.idleMode(IdleMode.kBrake);
    hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Expanding Hopper", 0);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rotations = hopperEncoder.getPosition();
    SmartDashboard.putNumber("Hopper Encoder", rotations);
  }

  // Methods
  // Set Power Method
    public void setHopperPower(double power) {
      hopperMotor.set(power);
    }
  // Set Position Method
    public void setHopperPosition(double position) {
      hopperMotor.set(position);
    }
  // Command Factory
  // Set Power Command
public Command setHopperCommand(HopperSubSystem hopperSubSystem, double speed) {
      return Commands.runEnd(() -> setHopperPower(speed), () -> setHopperPower(0));
    }
  // Set Position Command
  public Command setHopperPositionCommand(HopperSubSystem hopperSubSystem, double position) {
      return Commands.run(() -> hopperClosedLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0));
    }

}
