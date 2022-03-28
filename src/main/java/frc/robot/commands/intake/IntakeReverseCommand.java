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

  public IntakeReverseCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
    this.intake = intake;
    this.hopper = hopper;

    addRequirements(this.intake, this.hopper);
  }
    
  // TODO: If the stage or prestages are full run them in reverse to flush the system.
  @Override
  public void execute() {
    // Spin the intake wheels in reverse.
    intake.reverse();
    hopper.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    hopper.stop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}