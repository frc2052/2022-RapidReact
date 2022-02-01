// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class TwoWheelFly extends SubsystemBase {
  
  private TalonSRX topMotor;
  private TalonSRX bottomMotor;

  public TwoWheelFly() {
    topMotor = new TalonSRX(MotorIDs.TOP_SHOOTER_MOTOR);
    topMotor.configFactoryDefault();
    topMotor.setNeutralMode(NeutralMode.Coast);
    topMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    topMotor.setSelectedSensorPosition(0, 0, 10); 

    bottomMotor = new TalonSRX(MotorIDs.BOTTOM_SHOOTER_MOTOR);
    bottomMotor.configFactoryDefault();
    bottomMotor.setNeutralMode(NeutralMode.Coast);
    bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    bottomMotor.setSelectedSensorPosition(0, 0, 10);
  } 

  public void runAtShootSpeed(double topWheelSpeed, double bottomWheelSpeed) {
    topMotor.set(ControlMode.Velocity, topWheelSpeed);
    bottomMotor.set(ControlMode.Velocity, bottomWheelSpeed);
    //encoder needed for ticks per revolution calculation
  }
  
}
