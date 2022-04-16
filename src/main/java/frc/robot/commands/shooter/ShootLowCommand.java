// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootLowCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  public ShootLowCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(this.shooter, this.indexer);
  }

  @Override
  public void execute() {
    // Run the shooter at 20% while running the feeder and preloader.
    shooter.shootAtPercentage(20, 20);
    indexer.runFeeder();
    indexer.runPreload();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter, feeder, and preloader.
    shooter.stop();
    indexer.stopFeeder();
    indexer.stopPreload();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
