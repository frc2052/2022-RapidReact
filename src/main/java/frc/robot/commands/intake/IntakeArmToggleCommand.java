// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmToggleCommand extends CommandBase {
  private final IntakeSubsystem intake;

  /** 
   * Command to toggle the intake up or down 
   */
  public IntakeArmToggleCommand(IntakeSubsystem intake) {
    this.intake = intake;

    //addRequirements(this.intake); // Requirement commented out because unlikely this can conflict with anything and more likely the requirement itself would cause conflicts.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isArmOut()) {
      intake.armIn();
    } else {
      intake.armOut();
    }
  }

  @Override
  public boolean isFinished() { // Command ends immediatley
    return true;
  }
}
