// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedTwoCargoLaunchCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public FeedTwoCargoLaunchCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.isAtSpeed()) {
      indexerSubsystem.runFeeder();
      indexerSubsystem.runPreload();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
