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

public class Indexer extends SubsystemBase {
  
  private VictorSPX indexerMotor;

  public Indexer() {
    indexerMotor = new VictorSPX(MotorIDs.INDEXER_MOTOR);
    indexerMotor.configFactoryDefault();
    indexerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void forward() {
    indexerMotor.set(ControlMode.PercentOutput, Constants.ShooterSub.INDEXER_SPEED);
  }

  public void backward() {
    indexerMotor.set(ControlMode.PercentOutput, -Constants.ShooterSub.INDEXER_SPEED);
  }

  public void stop() {
    indexerMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
