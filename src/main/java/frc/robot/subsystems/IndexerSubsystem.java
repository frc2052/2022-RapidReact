// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class IndexerSubsystem extends SubsystemBase {
  
  private static VictorSPX largeIndexer;
  private static VictorSPX feederIndexer;
  private final DigitalInput preStagedCargoDetector = new DigitalInput(Constants.LimitSwitch.INDEXER_PRELOAD);
  private final DigitalInput stagedCargoDetector = new DigitalInput(Constants.LimitSwitch.INDEXER_FEEDER);

  public IndexerSubsystem() {
    largeIndexer = new VictorSPX(MotorIDs.INDEXER_MOTOR);
    largeIndexer.configFactoryDefault();
    largeIndexer.setNeutralMode(NeutralMode.Brake);
    feederIndexer = new VictorSPX(MotorIDs.INDEXER_MOTOR_FEEDER);
    feederIndexer.configFactoryDefault();
    feederIndexer.setNeutralMode(NeutralMode.Brake);
  }

  public void runPreload() {
    largeIndexer.set(ControlMode.PercentOutput, Constants.ShooterSub.INDEXER_SPEED);
  }

  public void runFeeder() {
    feederIndexer.set(ControlMode.PercentOutput, Constants.ShooterSub.FEEDER_SPEED);
  }

  public void stopPreload() {
    largeIndexer.set(ControlMode.PercentOutput, 0);
  }

  public void stopFeeder() {
    feederIndexer.set(ControlMode.PercentOutput, 0);
  }

  public double getLargeIndexerSpeed() {
    double largeIndexerRunning = largeIndexer.getSelectedSensorVelocity();
    return largeIndexerRunning;
  }

  public double getFeederIndexerSpeed() {
    double feederIndexerRunning = feederIndexer.getSelectedSensorVelocity();
    return feederIndexerRunning;
  }

  public boolean getCargoPreStagedDetected() {
    return preStagedCargoDetector.get();
  }

  public boolean getCargoStagedDetected() {
    return stagedCargoDetector.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feeder Indexer Speed", getFeederIndexerSpeed());
    SmartDashboard.putNumber("Large Indexer Speed", getLargeIndexerSpeed());
  }
  
}
