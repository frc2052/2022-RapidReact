// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;

/**
 * Creates a new ArmToggle.
 *
 * @param intakeSubsystem The subsystem used by this command.
 */
  public IntakeReverseCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
    this.intake = intake;
    this.hopper = hopper;

    addRequirements(intake, hopper);
  }
    
  @Override
  public void execute() {
    // arm extends and spins the wheels
    intake.intakeArmOut();
    intake.intakeReverse();
    hopper.hopperReverse();
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
    hopper.hopperStop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}