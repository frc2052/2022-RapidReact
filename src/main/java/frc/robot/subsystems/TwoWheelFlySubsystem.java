// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs; 

public class TwoWheelFlySubsystem extends SubsystemBase {
  private final TalonSRX topMotor;
  private final TalonSRX bottomMotor;
  private double topWheelVelocity;
  private double bottomWheelVelocity;

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

  public void runAtShootSpeed() {
    topMotor.set(ControlMode.Velocity, topWheelVelocity);
    bottomMotor.set(ControlMode.Velocity, bottomWheelVelocity);
  }

  public boolean isAtSpeed() {
    //Consider the shooter on target if we are within 5% of target speeds 
    boolean topIsOnTarget = topMotor.getSelectedSensorVelocity() > topWheelVelocity * .95 
      && topMotor.getSelectedSensorVelocity() < topWheelVelocity * 1.05;

    boolean bottomIsOnTarget = bottomMotor.getSelectedSensorVelocity() > bottomWheelVelocity * .95
      && bottomMotor.getSelectedSensorVelocity() < bottomWheelVelocity * 1.05; 

    return topIsOnTarget && bottomIsOnTarget;
  }

  public void setTopWheelVelocity(double velocity) {
    topWheelVelocity = velocity;
  }

  public void setBottomWheelVelocity(double velocity) {
    bottomWheelVelocity = velocity;
  }

  public void setBothWheelVelocities(double velocity) {
    topWheelVelocity = velocity;
    bottomWheelVelocity = velocity;
  }

  public void stop() {
    topMotor.set(ControlMode.Velocity, 0);
    bottomMotor.set(ControlMode.Velocity, 0);
  }

  public void putToSmartDashboard() {
    topWheelVelocity = SmartDashboard.getNumber("Top Wheel Velocity", 5);
    bottomWheelVelocity = SmartDashboard.getNumber("Bottom Wheel Velocity", 5);
    SmartDashboard.putBoolean("Shooter Wheels At Speed?", isAtSpeed());
    SmartDashboard.putNumber("Shooter Top Wheel Speed", topMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Bottom Wheel Speed", bottomMotor.getSelectedSensorVelocity());
  }

  //Putting this here until a better place is given - calculating needed velocity
  /**
   * Calculates required velocity to send a projectile to a point with given distance and height relative to launcher, as well as the angle to fire from
   * using the equation V0 = √((-9.8d²)/(2cos²(θ)(h-d*tanθ))).
   * @param distance
   * @return
   */
  public double calculateReqProjectileVelocity(double distanceMeters) {
    double height = Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.ShooterSub.SHOOTER_MOUNT_HEIGHT_METERS;
    double angleRadians = Math.toRadians(Constants.ShooterSub.SHOOTER_FIRING_ANGLE_DEGREES); 
    return Math.sqrt(-9.8 * (Math.pow(distanceMeters, 2)) / (2 * Math.pow(Math.cos(angleRadians), 2) * (height - distanceMeters * Math.tan(angleRadians))));
  }

  /**
   * Finds angular velocity and converts to RPM by diving by 2PI and multipying by 60.
   * @param velocity
   * @return
   */
  public double calculateReqShooterRPM(double velocity) {
    return velocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS / (2 * Math.PI) * 60; 
  }
}
