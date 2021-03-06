// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHopperRunCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  public IntakeHopperRunCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(indexer, hopper);
    this.intake = intake;
  
    //addRequirements(this.intake);
  }
    
  @Override
  public void execute() {
    super.execute();
    
    // Don't allow more balls to be picked up if both the stage and prestage are full (counted in the HopperBaseCommand isFinished method).
    if (!isFinished()) { // check isFinished first because execute is normally run before isfinished when command begins.
      intake.run();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.stop();
  }
}
