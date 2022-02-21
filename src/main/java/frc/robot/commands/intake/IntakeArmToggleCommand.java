// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmToggleCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  /** Creates a new IntakeArmToggleCommand. */
  public IntakeArmToggleCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(indexer, hopper);
    this.intake = intake;

    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isIntakeArmOut()) {
      intake.intakeArmIn();
    } else {
      intake.intakeArmOut();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
