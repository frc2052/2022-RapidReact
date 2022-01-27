// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerticalAngle extends SubsystemBase {
  
  private TalonSRX turretMotor;
  private int currentPos = 0;
  private double motorPower = 0;
  private boolean isLinedUp;

  public VerticalAngle() { 

  turretMotor = new TalonSRX(Constants.Motors.TURRET);      
  turretMotor.configFactoryDefault(); 
  turretMotor.setNeutralMode(NeutralMode.Brake);
  turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); //Selecting Feedback Sensor for Motor
  turretMotor.setSelectedSensorPosition(0, 0, 10); //Selected Sensor Position for Motor

  }

  public void resetEncoder() {

    turretMotor.setSelectedSensorPosition(0);

  }

  public void TurnTurret(double power) {
  motorPower = power;
  turretMotor.set(ControlMode.PercentOutput, power);

  }

  public void driveToPos(double angle) {

    //will take in angle and drive to find a shot

  }

  public boolean getIsOnTarget() {

    return isLinedUp;

  }

  public int getEncoderPos() {

    return (int) turretMotor.getSelectedSensorPosition();

  }

  public double getTurretDegree() {

    double ticks = turretMotor.getSelectedSensorPosition(0);
    return ticks; // Constants.Turret.kTicksPerDegree; Will return ticks per degree from constants

  }

  // public void printEncoderPos() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
