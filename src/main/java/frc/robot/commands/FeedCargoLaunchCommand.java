// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;

public class FeedCargoLaunchCommand extends CommandBase {

  private final TwoWheelFlySubsystem twoWheelFly;
  private final IndexerSubsystem indexer;

  public FeedCargoLaunchCommand(TwoWheelFlySubsystem twoWheelFly, IndexerSubsystem indexer) {
    this.twoWheelFly = twoWheelFly;
    this.indexer = indexer;
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (twoWheelFly.isAtSpeed()) {
      indexer.runPreload();  
      indexer.runFeeder();
    }
  }

  @Override
   public void end(boolean interrupted) {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
