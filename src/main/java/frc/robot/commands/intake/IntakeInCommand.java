// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  public IntakeInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
    super(indexer, grassHopper);
    this.intake = intake;
    
    addRequirements(this.intake);
  }
    
  @Override
  public void execute() {
    super.execute();
    
    // Don't allow more balls to be picked up if both the stage and prestage are full.
    if (!isFinished()) {
      intake.run();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.stop();
  }
  
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
