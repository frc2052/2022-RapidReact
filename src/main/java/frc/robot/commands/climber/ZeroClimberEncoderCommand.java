// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ZeroClimberEncoderCommand extends CommandBase {
  private final HookClimberSubsystem climber;
  
  /** Command to zero climber encoder */
  public ZeroClimberEncoderCommand(HookClimberSubsystem climber) {
    this.climber = climber;
  }

  @Override
  public void initialize() {
    climber.zeroEncoder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
