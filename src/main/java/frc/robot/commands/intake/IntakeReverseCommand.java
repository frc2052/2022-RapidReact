// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final HopperSubsystem grassHopper;

  public IntakeReverseCommand(IntakeSubsystem intake, HopperSubsystem grassHopper) {
    this.intake = intake;
    this.grassHopper = grassHopper;

    addRequirements(this.intake, this.grassHopper);
  }
    
  // TODO: If the stage or prestages are full run them in reverse to flush the system.
  @Override
  public void execute() {
    // Extends intake arm and spin the intake wheels in reverse.
    intake.intakeArmOut(); // TODO: Remove this line!
    intake.intakeReverse();
    grassHopper.hopperReverse();
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
    grassHopper.hopperStop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}