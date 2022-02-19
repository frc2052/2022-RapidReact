// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnInPlaceCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  // Angle change from current robot angle.
  private final Rotation2d deltaAngle;
  // Combined angle of robot and deltaAngle used to find.
  private Rotation2d targetTurn;

  /**
   * Creates a new turnInPlaceCommand.
   * 
   * @param drivetrain
   * @param deltaAngle angle that the robot should turn to (relative to robot).
   */
  public TurnInPlaceCommand(DrivetrainSubsystem drivetrain, Rotation2d deltaAngle) {
    this.drivetrain = drivetrain;
    this.deltaAngle = deltaAngle;
   
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Finds how far the robot needs to turn to meet the delta angle.
    targetTurn = drivetrain.getPose().getRotation().plus(deltaAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turns the drivetrain in place using 5% of a full rotation per second.
    // TODO: Use PID to control speed ramping and prevent overshooting.
    drivetrain.drive(new ChassisSpeeds(0, 0, Math.copySign(Math.PI, deltaAngle.getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when isFinished() returns true.
    drivetrain.stop();
  }

  // Returns true when the robot has turned to the correct rotation.
  @Override
  public boolean isFinished() {
    // Find the difference between the current rotation and our target angle (deltaAngle)
    // and check if the difference is less than 5 degrees inwhich case the robot angle is on target.
    return Math.abs(drivetrain.getPose().getRotation().minus(targetTurn).getDegrees()) <= 5;
  }
}