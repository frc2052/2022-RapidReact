// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedOneCargoLaunchCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public FeedOneCargoLaunchCommand (ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
  }

  @Override
  public void execute () {
    if (shooterSubsystem.isAtSpeed ()) {
      //Only the first ball in the shooter should be touching the feed wheel
      //If we have a second ball it should only be touching the green indexer wheel
      indexerSubsystem.runFeeder ();
    }
  }

  @Override
  public void end (boolean interrupted) {
    indexerSubsystem.stopFeeder ();
  }

  @Override
  public boolean isFinished () {
    return false;
  }
}
