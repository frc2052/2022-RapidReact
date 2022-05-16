// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class IndexerSubsystem extends SubsystemBase {
  
  private static TalonSRX preloadIndexerMotor;
  private static TalonSRX feederIndexerMotor;
  private final DigitalInput preStagedCargoDetector = new DigitalInput(Constants.LimitSwitch.INDEXER_PRELOAD);
  private final DigitalInput stagedCargoDetector = new DigitalInput(Constants.LimitSwitch.INDEXER_FEEDER);

  public IndexerSubsystem() {
    preloadIndexerMotor = new TalonSRX(MotorIDs.PRELOAD_INDEXER_MOTOR);
    preloadIndexerMotor.configFactoryDefault();
    preloadIndexerMotor.setNeutralMode(NeutralMode.Brake);

    feederIndexerMotor = new TalonSRX(MotorIDs.FEEDER_INDEXER_MOTOR);
    feederIndexerMotor.configFactoryDefault();
    feederIndexerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void runPreload() {
    //   /System.err.println("Running Preload");
    preloadIndexerMotor.set(ControlMode.PercentOutput, Constants.Shooter.PRELOAD_WHEEL_SPEED);
  }

  public void runPreloadReverse() {
    //System.err.println("Running Preload Reverse");
    preloadIndexerMotor.set(ControlMode.PercentOutput, -Constants.Shooter.PRELOAD_WHEEL_SPEED);
  }

  public void runFeeder() {
    //System.err.println("Running Feeder");
    feederIndexerMotor.set(ControlMode.PercentOutput, -Constants.Shooter.INDEXER_WHEEL_SPEED);
  }

  public void runFeederReverse() {
    //System.err.println("Running Feeder Reverse");
    feederIndexerMotor.set(ControlMode.PercentOutput, Constants.Shooter.INDEXER_WHEEL_SPEED);
  }

  public void stopPreload() {
    //System.err.println("Stopping Preload");
    preloadIndexerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopFeeder() {
    //System.err.println("Running Stopping Feeder");
    feederIndexerMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getLargeIndexerSpeed() {
    double largeIndexerRunning = preloadIndexerMotor.getSelectedSensorVelocity();
    return largeIndexerRunning;
  }

  public double getFeederIndexerSpeed() {
    double feederIndexerRunning = feederIndexerMotor.getSelectedSensorVelocity();
    return feederIndexerRunning;
  }

  public boolean getCargoPreStagedDetected() {
    //returns true if beam is not broken, no ball
    return !preStagedCargoDetector.get();
  }

  public boolean getCargoStagedDetected() {
    //returns true if beam is not broken, no ball
    return !stagedCargoDetector.get();
  }

  @Override
  public void periodic() {
    boolean stagedDetected = getCargoStagedDetected();
    boolean preStagedDetected = getCargoPreStagedDetected();

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Feeder Indexer Speed", getFeederIndexerSpeed());
    // SmartDashboard.putNumber("Large Indexer Speed", getLargeIndexerSpeed());
    SmartDashboard.putBoolean("Staged Cargo Detected", stagedDetected);
    SmartDashboard.putBoolean("PreStaged Cargo Detected", preStagedDetected);

    if (stagedDetected && preStagedDetected) {
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.HOPPER_FULL);
    } else if (!DriverStation.isDisabled() && (preStagedDetected || stagedDetected)) {
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.INTAKE_ON_1_BALL);
    }
  }
  
}
