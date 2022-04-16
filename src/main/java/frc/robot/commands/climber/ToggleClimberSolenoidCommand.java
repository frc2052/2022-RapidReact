// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HookClimberSubsystem.ClimberSolenoidState;

public class ToggleClimberSolenoidCommand extends CommandBase {
  private final HookClimberSubsystem climberSubsystem;

  public ToggleClimberSolenoidCommand(HookClimberSubsystem climbSubsystem) {
      this.climberSubsystem = climbSubsystem;

      addRequirements(this.climberSubsystem);
  }

  @Override
  public void initialize() {
    if (climberSubsystem.getClimberSolenoidState() == ClimberSolenoidState.FORWARD) {
      climberSubsystem.shiftBackward();
    } else {
      climberSubsystem.shiftForward();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
