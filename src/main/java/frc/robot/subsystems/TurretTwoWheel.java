// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretTwoWheel extends SubsystemBase {
  
  private TalonSRX angleMotor = null;

  public TurretTwoWheel() {

angleMotor = new TalonSRX(Constants.Motors.kHoodMotorID);      
  angleMotor.configFactoryDefault(); 
  angleMotor.setNeutralMode(NeutralMode.Brake);
  angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
  angleMotor.config_kF(0, 0.00001, 10); 
  angleMotor.config_kP(0, 0, 10);
  angleMotor.config_kI(0, 0, 10);
  angleMotor.config_kD(0, 0, 10);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
