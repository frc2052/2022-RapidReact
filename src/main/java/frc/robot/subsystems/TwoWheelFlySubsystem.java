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

public class TwoWheelFlySubsystem extends SubsystemBase {
  
  private static TalonSRX topMotor;
  private static TalonSRX bottomMotor;

  public TwoWheelFlySubsystem() {
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

  public boolean isAtSpeed() {
    //Consider the shooter on target if we are within 5% of target speeds 
    boolean topIsOnTarget = topMotor.getSelectedSensorVelocity() > Constants.ShooterSub.TOPWHEELSPEED * .95 
      && topMotor.getSelectedSensorVelocity() < Constants.ShooterSub.TOPWHEELSPEED * 1.05;

    boolean bottomIsOnTarget = bottomMotor.getSelectedSensorVelocity() > Constants.ShooterSub.BOTTOMWHEELSPEED * .95
      && bottomMotor.getSelectedSensorVelocity() < Constants.ShooterSub.BOTTOMWHEELSPEED * 1.05; 

    return topIsOnTarget && bottomIsOnTarget;
  }
  public void stop() {
    topMotor.set(ControlMode.Velocity, 0);
    bottomMotor.set(ControlMode.Velocity, 0);
  }

}
