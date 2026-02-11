// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Locale.IsoCountryCode;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelSubSystem extends SubsystemBase {

  // Fields for motor controllers and sensors related to the fuel system would be declared here
  private SparkMax feederRoller;
  private SparkMax launcherLeft;
  private SparkMax launcherRight;
  private SparkMax intakeMotor;

  private final RelativeEncoder m_leftLaunchEncoder;
  private final RelativeEncoder m_rightLaunchEncoder;

  /** Creates a new FuelSubSystem. */
  public FuelSubSystem() {

      // Initialize motor controllers and sensors here
      feederRoller = new SparkMax(Constants.FuelConstants.FUEL_FEEDER_ID, MotorType.kBrushless);
      launcherLeft = new SparkMax(Constants.FuelConstants.FUEL_SHOOTER_LEFT_ID, MotorType.kBrushless);
      launcherRight = new SparkMax(Constants.FuelConstants.FUEL_SHOOTER_RIGHT_ID, MotorType.kBrushless);
      intakeMotor = new SparkMax(Constants.FuelConstants.FUEL_INTAKE_ID, MotorType.kBrushless);
     
      // Setup Configuation for Intake and Feeder Motors
      SparkMaxConfig feederConfig = new SparkMaxConfig();
      feederConfig.smartCurrentLimit(Constants.FuelConstants.CURRENT_LIMIT);
      feederConfig.idleMode(IdleMode.kBrake);
      feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intakeMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Setup Configuration for Launcher Motors
      SparkMaxConfig launcherConfig = new SparkMaxConfig();
      launcherConfig.smartCurrentLimit(Constants.FuelConstants.CURRENT_LIMIT);
      launcherConfig.idleMode(IdleMode.kCoast);
      //launcherConfig.follow(launcherLeft, true);
      launcherRight.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //launcherConfig.disableFollowerMode();
      launcherLeft.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Encoders
      m_leftLaunchEncoder = launcherLeft.getEncoder();
      m_rightLaunchEncoder = launcherRight.getEncoder();

      m_leftLaunchEncoder.setPosition(0);
      m_rightLaunchEncoder.setPosition(0);

      // Add PID Control Later

      // Smart Dashboard
      SmartDashboard.putNumber("Left Launcher RPM", 0);
      SmartDashboard.putNumber("Right Launcher RPM:", 0);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the Velocities of the Launcher
    SmartDashboard.putNumber("Left Launcher RPM", m_leftLaunchEncoder.getVelocity());
    SmartDashboard.putNumber("Right Launcher RPM:", m_rightLaunchEncoder.getVelocity());

  }

  // Methods

  // Simple Turn on Left Launcher - will right follow?
  public void setLaunchPower(double power) {
    launcherLeft.set(-power);
    launcherRight.set(power);
  }

  // Stop Launcher
  public void stopLauncher() {
    launcherLeft.set(0);
    launcherRight.set(0);
  }

  public void setFeederPower(double power) {
    feederRoller.set(power);
  }

  // Command Factories

  // Test Commands to turn on and off the Launch Motors
  public Command launchSpeedCommand(FuelSubSystem fuelSubSystem, double speed) {
    return Commands.runEnd(() -> setLaunchPower(speed), () -> setLaunchPower(0), fuelSubSystem);
  }

  public Command stopLauncherCommand(FuelSubSystem fuelSubSystem) {
    return Commands.run(() -> stopLauncher());
  }

  public Command setFeederCommand(FuelSubSystem fuelSubSystem, double speed) {
    return Commands.runEnd(() -> setFeederPower(speed), () -> setFeederPower(0));
  }

}
