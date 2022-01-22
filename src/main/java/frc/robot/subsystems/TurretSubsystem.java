/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Copied from 2020 Scorpion for testing aiming using limelight

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.lib.CsvLogger;

public class TurretSubsystem extends SubsystemBase {
  private TalonSRX turretMotor;
  private int currentPos = 0;
  private double motorPower = 0;
  
  private boolean isLinedUp;
 
  public TurretSubsystem() {
    turretMotor = new TalonSRX(Constants.Motors.kTurretMotorID);
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    turretMotor.setSelectedSensorPosition(0, 0, 10);

    //CsvLogger.addLoggingFieldDouble("TurretTicks", "ticks", "getEncoderPos", this);
    //CsvLogger.addLoggingFieldBoolean("TurretOnTarget", "", "getIsOnTarget", this);
  }


  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
  }


  public void turnTurret(double power){
    //motorPower = power;
    turretMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // currentPos = turretMotor.getSelectedSensorPosition();
    // if (currentPos > Constants.Turret.kTurretMaxRight && motorPower > 0){
    //   motorPower = 0;
    // } else if (currentPos < Constants.Turret.kTurretMinLeft && motorPower < 0){
    //   motorPower = 0;
    // }
    // turretMotor.set(ControlMode.PercentOutput, motorPower);

    if(turretMotor.getSelectedSensorVelocity() < 0 && turretMotor.getSelectedSensorPosition() < Constants.Turret.kTurretMinEncoderPos) {
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else if(turretMotor.getSelectedSensorVelocity() > 0 && turretMotor.getSelectedSensorPosition() > Constants.Turret.kTurretMaxEncoderPos) {
      turretMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  
  // takes angle and drives until it gets to angle
  public void driveToPos(double angle) {
    angle = -angle;
    isLinedUp = false;
    //System.out.println(turretMotor.getSelectedSensorPosition());
    printEncoderPos();
    //System.out.println("****************** TARGET ANGLE: " + angle);
    if(angle < 0 && turretMotor.getSelectedSensorPosition() < Constants.Turret.kTurretMinEncoderPos) {
      //too far to the negative can't keep going
      System.out.println("TURRET TOO FAR NEGATIVE");
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else if (angle > 0 && turretMotor.getSelectedSensorPosition() > Constants.Turret.kTurretMaxEncoderPos) {
      //too far to the positive can't keep going
      System.out.println("TURRET TOO FAR POSITIVE");
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else {
      //System.out.println("TURRENT ANGLE OFFSET: " + angle);
      isLinedUp = Math.abs(angle) < 2;
      if(Math.abs(angle) < 0.5) {
        turretMotor.set(ControlMode.PercentOutput, 0);
      } else {
        if (Math.abs(angle) >= 20) {
          isLinedUp = false;
          if(angle > 0) {
            turretMotor.set(ControlMode.PercentOutput, .75);
          } else {
            turretMotor.set(ControlMode.PercentOutput, -.75);
          }
        } else if (Math.abs(angle) < 20) {
          isLinedUp = false;
          double targetSpeed = angle * .05;

          if(Math.abs(targetSpeed) < Constants.Turret.kMinTurretSpeed) {
            if (targetSpeed < 0) {
              targetSpeed = -Constants.Turret.kMinTurretSpeed;
            } else {
              targetSpeed = Constants.Turret.kMinTurretSpeed;
            }
          }

          turretMotor.set(ControlMode.PercentOutput, (targetSpeed ));
        }
      }
    }
  }

  public boolean getIsOnTarget() {
    return isLinedUp;
  }

  public double getEncoderPos() {
    return turretMotor.getSelectedSensorPosition();
  }

  public double getTurretDegree() {
    double ticks = turretMotor.getSelectedSensorPosition(0);
    return ticks / Constants.Turret.kTicksPerDegree;
  }

  public void printEncoderPos() {
    SmartDashboard.putNumber("TURRET ENCODER", turretMotor.getSelectedSensorPosition());
  }
}