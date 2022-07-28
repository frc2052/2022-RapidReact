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
  private final Rotation2d deltaAngle;

  // Combined angle of robot and deltaAngle used to find.
  private Rotation2d targetTurn;

  /**
   * Command for turning the robot a certain rotation angle (not field relative).
   * @param drivetrain
   * @param deltaAngle - Rotation2d of the angle change from current robot angle.

   */
  public TurnInPlaceCommand(DrivetrainSubsystem drivetrain, Rotation2d deltaAngle) {
    this.drivetrain = drivetrain;
    this.deltaAngle = deltaAngle;
   
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    targetTurn = drivetrain.getPose().getRotation().plus(deltaAngle); // Finds how far the robot needs to turn to meet the delta angle.
  }

  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(0, 0, Math.copySign(Math.PI, deltaAngle.getDegrees()))); // Turns the drivetrain in place using 5% of a full rotation per second.
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop(); // Stop the drivetrain when isFinished() returns true.
  }

  @Override
  public boolean isFinished() {
    // Find the difference between the current rotation and our target angle (deltaAngle)
    // and check if the difference is less than 5 degrees inwhich case the robot angle is on target.
    return Math.abs(drivetrain.getPose().getRotation().minus(targetTurn).getDegrees()) <= 5;
  }
}
