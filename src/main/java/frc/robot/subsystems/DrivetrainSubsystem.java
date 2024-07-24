// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team2052.swervemodule.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SwerveModule frontLeftWheel;
  private final SwerveModule frontRightWheel;
  private final SwerveModule backLeftWheel;
  private final SwerveModule backRightWheel;

  public double xMetersPerSecond;
  public double yMetersPerSecond;
  public double radiansPersecond;

  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {

    frontLeftWheel = new SwerveModule(
      "frontLeftModule",
      Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.FRONT_LEFT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.FRONT_LEFT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_ANGLE_OFFSET_RADIANS)
    );
    frontRightWheel = new SwerveModule(
      "frontRightModule",
      Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_ANGLE_OFFSET_RADIANS)
    );
    backLeftWheel = new SwerveModule(
      "backLeftModule",
      Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.BACK_LEFT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.BACK_LEFT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_ANGLE_OFFSET_RADIANS)

    );
    backRightWheel = new SwerveModule(
      "backRightModule",
      Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.BACK_RIGHT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.BACK_RIGHT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_ANGLE_OFFSET_RADIANS)
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  }
    /**
   * All parameters are taken in normalized terms of [-1.0 to 1.0].
   */
  public void drive (double normalizedXVelocity, double normalizedYVelocity, double normalizedRotationVelocity){
    
    normalizedXVelocity = Math.copySign(
      Math.min(Math.abs(normalizedXVelocity), 1.0),
      normalizedXVelocity
    );
    normalizedYVelocity = Math.copySign(
      Math.min(Math.abs(normalizedYVelocity), 1.0),
      normalizedYVelocity
    );
    normalizedRotationVelocity = Math.copySign(
      Math.min(Math.abs(normalizedRotationVelocity), 1.0),
      normalizedRotationVelocity
    );

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      normalizedXVelocity * SwerveModule.getMaxVelocityMetersPerSecond(), 
      normalizedYVelocity * SwerveModule.getMaxVelocityMetersPerSecond(), 
      normalizedRotationVelocity * SwerveModule.getMaxAngularVelocityRadiansPerSecond()
    );

    drive(chassisSpeeds);
  }

  // Stop wheels from snapping to 0 when they have no velocity
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
    boolean hasVelocity = swerveModuleStates[0].speedMetersPerSecond != 0
        || swerveModuleStates[1].speedMetersPerSecond != 0 
        || swerveModuleStates[2].speedMetersPerSecond != 0
        || swerveModuleStates[3].speedMetersPerSecond != 0;
    frontLeftWheel.setState(
        swerveModuleStates[0].speedMetersPerSecond, 
        hasVelocity ? swerveModuleStates[0].angle : frontRightWheel.getState().angle
    );
    frontRightWheel.setState(
        swerveModuleStates[1].speedMetersPerSecond, 
        hasVelocity ? swerveModuleStates[1].angle : frontLeftWheel.getState().angle
    );    
    backLeftWheel.setState(
        swerveModuleStates[2].speedMetersPerSecond, 
        hasVelocity ? swerveModuleStates[2].angle : backRightWheel.getState().angle
    );
    backRightWheel.setState(
        swerveModuleStates[3].speedMetersPerSecond, 
        hasVelocity ? swerveModuleStates[3].angle : backLeftWheel.getState().angle
    );
  }

  public void stop() {
    drive(0, 0, 0);
  }

  @Override
  public void periodic() {
    debug();
  }

  public void debug() {
    backLeftWheel.debug("Back Left");
    backRightWheel.debug("Back Right");
    frontLeftWheel.debug("Front Left");
    frontRightWheel.debug("Front Right");
  }
}
