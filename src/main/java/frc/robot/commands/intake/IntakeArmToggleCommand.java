// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmToggleCommand extends HopperBaseCommand {
  private final IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeArmToggleCommand. */
  public IntakeArmToggleCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, HopperSubsystem hopperSubsystem) {
    super(indexerSubsystem, hopperSubsystem);
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
    hopperSubsystem.hopperStop();

    if (intakeSubsystem.isIntakeArmOut()) {
      intakeSubsystem.intakeArmIn();
    } else {
      intakeSubsystem.intakeArmOut();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
