// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class IndexerSubsystem extends SubsystemBase {
  
  private static VictorSPX largeIndexer;
  private static VictorSPX feederIndexer;

  public IndexerSubsystem() {
    largeIndexer = new VictorSPX(MotorIDs.INDEXER_MOTOR);
    largeIndexer.configFactoryDefault();
    largeIndexer.setNeutralMode(NeutralMode.Brake);
    feederIndexer = new VictorSPX(MotorIDs.INDEXER_MOTOR_FEEDER);
    feederIndexer.configFactoryDefault();
    feederIndexer.setNeutralMode(NeutralMode.Brake);
    //TODO: Cry, Limit Switch, Victor;
  }

  public void runPreload() {
    largeIndexer.set(ControlMode.PercentOutput, Constants.ShooterSub.INDEXER_SPEED);
  }

  public void runFeeder() {
    feederIndexer.set(ControlMode.PercentOutput, Constants.ShooterSub.FEEDER_SPEED);
  }

  public void stop() {
    largeIndexer.set(ControlMode.PercentOutput, 0);
    feederIndexer.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
