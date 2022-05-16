// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WaitOdometryResetCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;

  /** 
   * Command intended to check and make sure the robot's odometry was completley at 0 before 
   * starting an auto, because we had issues with odometry messing up the path if another auto
   * was initiated without redeploying or power cycling.
   * Did not work.
   */
  public WaitOdometryResetCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPos = drivetrain.getPose();
    if (Math.abs(currentPos.getX()) > .05 || Math.abs(currentPos.getX()) > .05 ) { //no more than 5cm
      System.err.println("$$$$$$$$$$$$$$$$$$$   Odometry not reset  $$$$$$$$$$$$$$$$$$$$$$$$$$$");
      drivetrain.zeroGyroscope();
      return false;
    } else {
      return true;
    }
  }
}
