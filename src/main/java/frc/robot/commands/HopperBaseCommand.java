// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;

public class HopperBaseCommand extends CommandBase {
  protected final TwoWheelFlySubsystem twoWheelFly; 
  protected final IndexerSubsystem indexer;
  protected final HopperSubsystem hopper;
  
  /** Creates a new HopperBaseCommand. */
  public HopperBaseCommand(TwoWheelFlySubsystem twoWheelFly, IndexerSubsystem indexer, HopperSubsystem hopper) {
    this.twoWheelFly = twoWheelFly;
    this.indexer = indexer;
    this.hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
