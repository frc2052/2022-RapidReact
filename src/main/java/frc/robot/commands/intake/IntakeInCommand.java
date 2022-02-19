// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends HopperBaseCommand {
  private final IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public IntakeInCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, HopperSubsystem hopperSubsystem) {
    super(indexerSubsystem, hopperSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    
    addRequirements(intakeSubsystem);
  }
    
  @Override
  public void initialize() {
    // Arm extends and spins the wheels
    intakeSubsystem.intakeArmOut();
    intakeSubsystem.intakeOn();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
