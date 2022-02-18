// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final HopperSubsystem hopperSubsystem;

/**
 * Creates a new ArmToggle.
 *
 * @param intakeSubsystem The subsystem used by this command.
 */
  public IntakeReverseCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.hopperSubsystem = hopperSubsystem;

    addRequirements(intakeSubsystem, hopperSubsystem);
  }
    
  @Override
  public void execute() {
    intakeSubsystem.intakeArmOut();
    intakeSubsystem.intakeReverse();
    hopperSubsystem.hopperReverse();
    //arm extends and spins the wheels
  }
  @Override
  public boolean isFinished(){
    return true;
  }

}