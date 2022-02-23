// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WaitOdometryResetCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  /** Creates a new WaitOdometryResetCommand. */
  public WaitOdometryResetCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d current = drivetrain.getPose();
    if (Math.abs(current.getX()) > .05 || Math.abs(current.getX()) > .05 ) { //no more than 5cm
      System.err.println("$$$$$$$$$$$$$$$$$$$   Odometry not reset  $$$$$$$$$$$$$$$$$$$$$$$$$$$");
      drivetrain.zeroGyroscope();
      return false;
    } else {
      return true;
    }
  }
}
