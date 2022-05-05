// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final DoubleSolenoid angleChangeSolenoid;

  private double topWheelTargetVelocity;
  private double bottomWheelTargetVelocity;
  private double shooterVelocityBoost;
  private FiringAngle currentAngle;
  private boolean shootAngle1Override;
  private boolean idleSpeedEnabled;

//   private double p;
//   private double p;

//   private double p;
//   private double p;


  public ShooterSubsystem() {
    topMotor = new TalonFX(MotorIDs.TOP_SHOOTER_MOTOR);
    topMotor.configFactoryDefault();
    topMotor.setNeutralMode(NeutralMode.Coast);
    topMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // topMotor.setSelectedSensorPosition(0, 0, 10);
    // topMotor.config_kP(0, 1, 10);
    topMotor.config_kP(0, 0.11, 10);
    topMotor.config_kI(0, 0.001, 10); 
    topMotor.config_kD(0, 0.7, 10);
    // topMotor.config_kP(0, 0.1, 10);
    // topMotor.config_kI(0, 0.0004, 10);
    // topMotor.config_kD(0, 0.1, 10);
    //topMotor.config_kF(0, 0.048, 10); // PID Feed forward value - 12 (volts) devided by Maximum RPM - Maximum ticks per 100ms / 2048 (ticks per 1 revolution of a falcon) * 600 (amount of 100ms in a minute)
    //topMotor.configAllowableClosedloopError(0, 20, 10);

    bottomMotor = new TalonFX(MotorIDs.BOTTOM_SHOOTER_MOTOR);
    bottomMotor.configFactoryDefault();
    bottomMotor.setNeutralMode(NeutralMode.Coast);
    bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // bottomMotor.setSelectedSensorPosition(0, 0, 10);
    bottomMotor.config_kP(0, 0.11, 10);
    bottomMotor.config_kI(0, 0.001, 10);
    bottomMotor.config_kD(0, 0.7, 10);
    // bottomMotor.config_kP(0, 0.05, 10);
    // bottomMotor.config_kI(0, 0, 10);
    // bottomMotor.config_kD(0, 0, 10);
    // bottomMotor.config_kF(0, 0.0522, 10);

    angleChangeSolenoid = new DoubleSolenoid(
      Constants.Solenoids.COMPRESSOR_MODULE_ID,
      PneumaticsModuleType.REVPH,
      Constants.Solenoids.SHOOTER_ANGLE_IN_SOLENOID,
      Constants.Solenoids.SHOOTER_ANGLE_OUT_SOLENOID
    );

    currentAngle = angleChangeSolenoid.get() == Value.kReverse ? FiringAngle.ANGLE_2 : FiringAngle.ANGLE_1;

    // SmartDashboard.putNumber("Shooter P", 0.115);
    // SmartDashboard.putNumber("Shooter I", 0);
    // SmartDashboard.putNumber("Shooter D", 0);
    // SmartDashboard.putNumber("Shooter F", 0.048);
    idleSpeedEnabled = !SmartDashboard.getBoolean("Disable Shooter Idle", false);
    stop();
  }

//   public void updatePIDF(double p, double i, double d, double f) {
//     topMotor.config_kP(0, p, 10);
//     topMotor.config_kI(0, i, 10);
//     topMotor.config_kD(0, d, 10);
//     topMotor.config_kF(0, f, 10); 

//   }

  public void runAtSpeed(double topVelocityTicksPerSeconds, double bottomVelocityTicksPerSeconds) {
    //System.err.println("***************************** RUNNING SHOOTER");
    //System.err.println("***************************** TOP: " + topVelocityTicksPerSeconds);
    //System.err.println("***************************** BOTTOM: " + bottomVelocityTicksPerSeconds);
    topWheelTargetVelocity = topVelocityTicksPerSeconds * (shooterVelocityBoost + 1);
    bottomWheelTargetVelocity = bottomVelocityTicksPerSeconds * (shooterVelocityBoost + 1);

    topMotor.set(ControlMode.Velocity, topWheelTargetVelocity);// * Constants.Shooter.SHOOTER_TOP_PULLDOWN_PCT);
    bottomMotor.set(ControlMode.Velocity, bottomWheelTargetVelocity);// * Constants.Shooter.SHOOTER_BOTTOM_PULLDOWN_PCT);
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
    if (topWheelTargetVelocity != 0 && bottomWheelTargetVelocity != 0) { // To make sure indexing doesn't move balls into shooter because we're in fact at target speed 0
      boolean topIsOnTarget = topMotor.getSelectedSensorVelocity() > topWheelTargetVelocity * (1 - Constants.Shooter.SHOOTER_TOLERANCE)
          && topMotor.getSelectedSensorVelocity() < topWheelTargetVelocity * (1 + Constants.Shooter.SHOOTER_TOLERANCE);

      boolean bottomIsOnTarget = bottomMotor.getSelectedSensorVelocity() > bottomWheelTargetVelocity * (1 - Constants.Shooter.SHOOTER_TOLERANCE)
          && bottomMotor.getSelectedSensorVelocity() < bottomWheelTargetVelocity * (1 + Constants.Shooter.SHOOTER_TOLERANCE);

      return topIsOnTarget && bottomIsOnTarget;
    } else {
      return false;
    }

    // if (topIsOnTarget && bottomIsOnTarget) {
    //     if (timer == null) {
    //         timer = new Timer();
    //         timer.start();
    //     }
    //     if (timer.hasElapsed(0.25)) {
    //         return true;
    //     }
    // } else {
    //     clearTimer();
    // }
    // return false;
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
    //System.err.println("***************************** STOPPING SHOOTER");
    if (idleSpeedEnabled) { // Implemented idle speed not long before champs, so this was the easy solution to that rather than rewriting much more
      runAtSpeed(7400, 7400);
    } else {
      topWheelTargetVelocity = 0;
      bottomWheelTargetVelocity = 0;
      topMotor.set(ControlMode.PercentOutput, 0);
      bottomMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void shootAtPercentage(double topWheelPercent, double bottomWheelPercent) {
    //System.err.println("***************************** RUN BY PERCENT SHOOTER");
    topMotor.set(ControlMode.PercentOutput, topWheelPercent/100);
    bottomMotor.set(ControlMode.PercentOutput, bottomWheelPercent/100);
  }

  public void setIdleSpeedEnabled(boolean enabled) {
    idleSpeedEnabled = enabled;
    SmartDashboard.putBoolean("Disable Shooter Idle", !enabled);
  }

  public void setShooterVelocityBoost(double boostPct) {
    if (boostPct < -0.1) {
      boostPct = -0.1;
    } else if (boostPct > 0.1) {
      boostPct = 0.1;
    }
    shooterVelocityBoost = boostPct;
  }

  public void setShootAngle1() {
    if (!shootAngle1Override) {
      angleChangeSolenoid.set(Value.kReverse);
      currentAngle = FiringAngle.ANGLE_1;
    }
    // System.err.println("Setting Angle 1");
  }

  public void setShootAngle2() {
    if (!shootAngle1Override) {
      angleChangeSolenoid.set(Value.kForward);
      currentAngle = FiringAngle.ANGLE_2;
    }
    // System.err.println("Setting Angle 2");
  }

  public void setShootAngle1Override(boolean override) {
    shootAngle1Override = override;
  }

  public double getShootAngleDegrees() {
    return currentAngle.getAngleDegrees();
  }

  public FiringAngle getShootAngleEnum() {
    return currentAngle;
  }

  public double getTopWheelVelocity() {
    return topMotor.getSelectedSensorVelocity();
  }

  public double getBottomWheelVelocity() {
    return bottomMotor.getSelectedSensorVelocity();
  }

  public double getTargetTopWheelVelocity() {
      return topWheelTargetVelocity;
  }

  public double getTargetBottomWheelVelocity() {
    return bottomWheelTargetVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Wheels At Speed?", isAtSpeed());
    SmartDashboard.putNumber("Shooter Target Top Wheel Speed", topWheelTargetVelocity);
    SmartDashboard.putNumber("Shooter Target Bottom Wheel Speed", bottomWheelTargetVelocity);
    SmartDashboard.putNumber("Shooter Top Wheel Speed", topMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Bottom Wheel Speed", bottomMotor.getSelectedSensorVelocity());

    // SmartDashboard.putString("Firing Angle", currentAngle.toString());
    // SmartDashboard.putBoolean("Angle Override", shootAngle1Override);
  }

  public enum FiringAngle {
    ANGLE_1(Constants.Shooter.FIRING_ANGLE_1_DEGREES),
    ANGLE_2(Constants.Shooter.FIRING_ANGLE_2_DEGREES);

    private double angleDegrees;

    FiringAngle(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }
  }

  // private void clearTimer() {
  //   if(timer != null) {
  //       timer.stop();
  //       timer = null;
  //   }
  // }
}
