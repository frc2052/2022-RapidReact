// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OnlyIntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;

  public OnlyIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    
    addRequirements(this.intake);
  }
    
  @Override
  public void initialize() {
    // Extend intake arm and spin the intake wheels.
    intake.intakeArmOut(); // TODO: Remove this line!
    // Don't allow more balls to be picked up if both the stage and prestage are full.
    if (!isFinished()) {
      intake.intakeOn();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
