// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerticalAngle extends SubsystemBase {
  
  private TalonSRX pitchMotor;

  public VerticalAngle() { 
    pitchMotor = new TalonSRX(Constants.MotorIDs.PITCH_MOTOR);      
    pitchMotor.configFactoryDefault(); 
    pitchMotor.setNeutralMode(NeutralMode.Brake);
    pitchMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); //Selecting Feedback Sensor for Motor
    pitchMotor.setSelectedSensorPosition(0, 0, 10); //Selected Sensor Position for Motor
  }

  public void resetEncoder() {
    pitchMotor.setSelectedSensorPosition(0);
  }

  public void aimTurret(double ticks) {
    pitchMotor.set(ControlMode.Position, ticks);
  }

  public int getEncoderPos() {
    return (int) pitchMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
