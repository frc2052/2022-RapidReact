// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class WheelsFlyWheel extends SubsystemBase {
  
  private TalonSRX wheelsMotor;
  private double motorSpeed = 0; 


  public WheelsFlyWheel() {

    wheelsMotor = new TalonSRX(MotorIDs.FLYWHEELS_SHOOTER);
    wheelsMotor.configFactoryDefault();
    wheelsMotor.setNeutralMode(NeutralMode.Brake);
    wheelsMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    wheelsMotor.setSelectedSensorPosition(0, 0, 10);

  }

  public void runAtShootSpeed() {

    wheelsMotor.set(ControlMode.PercentOutput, Constants.ShooterSub.FLYWHEELSPEED);

  }

  //To Do: Cry, 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
