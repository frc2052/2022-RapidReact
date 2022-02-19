// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private double topWheelTargetVelocity;
  private double bottomWheelTargetVelocity;

  public ShooterSubsystem() {
    topMotor = new TalonFX(MotorIDs.TOP_SHOOTER_MOTOR);
    topMotor.configFactoryDefault();
    topMotor.setNeutralMode(NeutralMode.Coast);
    topMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // topMotor.setSelectedSensorPosition(0, 0, 10);
    topMotor.config_kP(0, 0.05, 10);
    topMotor.config_kI(0, 0, 10);
    topMotor.config_kD(0, 0, 10);
    topMotor.config_kF(0, 0.0522, 10);

    bottomMotor = new TalonFX(MotorIDs.BOTTOM_SHOOTER_MOTOR);
    bottomMotor.configFactoryDefault();
    bottomMotor.setNeutralMode(NeutralMode.Coast);
    bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // bottomMotor.setSelectedSensorPosition(0, 0, 10);
    bottomMotor.config_kP(0, 0.05, 10);
    bottomMotor.config_kI(0, 0, 10);
    bottomMotor.config_kD(0, 0, 10);
    bottomMotor.config_kF(0, 0.0522, 10);
  }

  public void shootAtSpeed(double topVelocityTicksPerSeconds, double bottomVelocityTicksPerSeconds) {
    System.err.println("***************************** RUNNING SHOOTER");
    System.err.println("***************************** TOP: " + topVelocityTicksPerSeconds);
    System.err.println("***************************** BOTTOM: " + bottomVelocityTicksPerSeconds);
    topWheelTargetVelocity = topVelocityTicksPerSeconds;
    bottomWheelTargetVelocity = bottomVelocityTicksPerSeconds;

    topMotor.set(ControlMode.Velocity, topWheelTargetVelocity);
    bottomMotor.set(ControlMode.Velocity, bottomWheelTargetVelocity);
  }

  // public void runAtShootSpeed() {
  //   topMotor.set(ControlMode.Velocity, topWheelVelocity);
  //   bottomMotor.set(ControlMode.Velocity, bottomWheelVelocity);
  // }

  // public void runAtShootSpeed(double topVelocityTicksPerSeconds, double bottomVelocityTicksPerSeconds) {
  //   setTopWheelVelocity(topVelocityTicksPerSeconds);
  //   setBottomWheelVelocity(bottomVelocityTicksPerSeconds);
  //   runAtShootSpeed();
  // }

  public boolean isAtSpeed() {
    // Consider the shooter on target if we are within a shooter tolerance of target speeds
    boolean topIsOnTarget = topMotor.getSelectedSensorVelocity() > topWheelTargetVelocity * (1 - Constants.ShooterSub.SHOOTER_TOLERANCE)
        && topMotor.getSelectedSensorVelocity() < topWheelTargetVelocity * (1 + Constants.ShooterSub.SHOOTER_TOLERANCE);

    boolean bottomIsOnTarget = bottomMotor.getSelectedSensorVelocity() > bottomWheelTargetVelocity * (1 - Constants.ShooterSub.SHOOTER_TOLERANCE)
        && bottomMotor.getSelectedSensorVelocity() < bottomWheelTargetVelocity * (1 + Constants.ShooterSub.SHOOTER_TOLERANCE);

    return topIsOnTarget && bottomIsOnTarget;
  }

  // public void setTopWheelVelocity(double velocity) {
  //   topWheelVelocity = velocity;
  // }

  // public void setBottomWheelVelocity(double velocity) {
  //   bottomWheelVelocity = velocity;
  // }

  // public void setBothWheelVelocities(double velocity) {
  //   topWheelVelocity = velocity;
  //   bottomWheelVelocity = velocity;
  // }

  public void stop() {
    System.err.println("***************************** STOPPING SHOOTER");
    topWheelTargetVelocity = 0;
    bottomWheelTargetVelocity = 0;

    topMotor.set(ControlMode.PercentOutput, 0);
    bottomMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runByPercentage(double bottomWheelPercent, double topWheelPercent) {
    System.err.println("***************************** RUN BY PERCENT SHOOTER");
    topMotor.set(ControlMode.PercentOutput, topWheelPercent);
    bottomMotor.set(ControlMode.PercentOutput, bottomWheelPercent);
  }

  public void putToSmartDashboard() {
    SmartDashboard.putBoolean("Shooter Wheels At Speed?", isAtSpeed());
    SmartDashboard.putNumber("Shooter Target Top Wheel Speed", topWheelTargetVelocity);
    SmartDashboard.putNumber("Shooter Target Bottom Wheel Speed", bottomWheelTargetVelocity);
    SmartDashboard.putNumber("Shooter Top Wheel Speed", topMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Bottom Wheel Speed", bottomMotor.getSelectedSensorVelocity());
  }
}
