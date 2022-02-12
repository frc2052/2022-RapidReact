// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ClimberSolenoidForwardCommand extends CommandBase {
  private final HookClimberSubsystem climberSubsystem;

  public ClimberSolenoidForwardCommand(HookClimberSubsystem climbSubsystem) {
      this.climberSubsystem = climbSubsystem;

      addRequirements(this.climberSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Push climber solenoid forward.
    climberSubsystem.shiftClimberForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
