// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ClimberSolenoidBackCommand extends CommandBase {
  private final HookClimberSubsystem climberSubsystem;

  public ClimberSolenoidBackCommand(HookClimberSubsystem climbSubsystem) {
      this.climberSubsystem = climbSubsystem;

      addRequirements(this.climberSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Slightly extend the climber then push the climber solenoid back.
    climberSubsystem.setArmPostionInches(7.5);
    climberSubsystem.shiftClimberForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.isAtDesiredPosition();
  }
}
