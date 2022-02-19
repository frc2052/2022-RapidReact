// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  /**
   * Creates a new ArmToggle.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public IntakeInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(indexer, hopper);
    this.intake = intake;
    
    addRequirements(intake);
  }
    
  @Override
  public void initialize() {
    // Arm extends and spins the wheels
    intake.intakeArmOut();
    if (!isFinished()) {
      intake.intakeOn();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
