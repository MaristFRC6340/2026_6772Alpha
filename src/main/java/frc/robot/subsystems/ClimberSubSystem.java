// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubSystem extends SubsystemBase {

  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;

  private SparkClosedLoopController climberClosedLoopController;
  private SparkMaxConfig climberConfig;


  /** Creates a new ClimberSubSystem. */
  public ClimberSubSystem() {
    climberMotor = new SparkMax(Constants.FuelConstants.CLIMBER_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberClosedLoopController = climberMotor.getClosedLoopController();

    climberConfig = new SparkMaxConfig();
    climberConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    climberConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-.95, .95);

    // Reset to Zero
    climberEncoder.setPosition(0);

    climberConfig.smartCurrentLimit(Constants.FuelConstants.CURRENT_LIMIT);
    climberConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Encoder of Climber to Smart Dashboard
    SmartDashboard.putNumber("Climber Encoder", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rotations = climberEncoder.getPosition();
    SmartDashboard.putNumber("Climber Encoder", rotations);

  }

  public void setClimberPower(double power) {
    climberMotor.set(power);
  }

  public Command setClimberCommand(ClimberSubSystem climberSubSystem, double speed) {
    return Commands.runEnd(() -> setClimberPower(speed), () -> setClimberPower(0));
  }

  public Command setClimberPositionCommand(ClimberSubSystem climberSubSystem, double position) {
    return Commands.run(() -> climberClosedLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0));
  }


}
