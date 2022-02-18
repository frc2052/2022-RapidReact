// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;


public class HopperBaseCommand extends CommandBase {
  protected final IndexerSubsystem indexerSubsystem;
  protected final HopperSubsystem hopperSubsystem;
  
  /** Creates a new HopperBaseCommand. */
  public HopperBaseCommand(IndexerSubsystem indexerSubsystem, HopperSubsystem hopperSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.hopperSubsystem = hopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.err.println("********************************" + indexerSubsystem.getCargoStagedDetected() + " ** " + indexerSubsystem.getCargoPreStagedDetected());

    if (!indexerSubsystem.getCargoStagedDetected()) {
      //Keep running all the wheels until all the balls are staged
      indexerSubsystem.runPreload();
      indexerSubsystem.runFeeder();
      hopperSubsystem.hopperGo();
    } else if (indexerSubsystem.getCargoStagedDetected() && !indexerSubsystem.getCargoPreStagedDetected()) {
      //The staged detector shows a ball ready to be fired but no second ball is detected
      indexerSubsystem.stopFeeder();
      indexerSubsystem.runPreload();
      hopperSubsystem.hopperGo();
    } else {
      //Two balls are loaded and no more can be taken
      indexerSubsystem.stopFeeder();
      indexerSubsystem.stopPreload();
      hopperSubsystem.hopperStop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
    hopperSubsystem.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
