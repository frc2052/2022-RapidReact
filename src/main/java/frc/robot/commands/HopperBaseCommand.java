// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class HopperBaseCommand extends CommandBase {
  protected final IndexerSubsystem indexer;
  protected final HopperSubsystem hopper;
  
  /** Creates a new HopperBaseCommand. */
  public HopperBaseCommand(IndexerSubsystem indexer, HopperSubsystem hopper) {
    this.indexer = indexer;
    this.hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!indexer.getCargoStagedDetected()) {
      //Keep running all the wheels until all the balls are staged
      indexer.runPreload();
      indexer.runFeeder();
      hopper.hopperGo();
    } else if (indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
      //The staged detector shows a ball ready to be fired but no second ball is detected
      indexer.stopFeeder();
      indexer.runPreload();
      hopper.hopperGo();
    } else {
      //Two balls are loaded and no more can be taken
      indexer.stopFeeder();
      indexer.stopPreload();
      hopper.hopperStop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopFeeder();
    indexer.stopPreload();
    hopper.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
