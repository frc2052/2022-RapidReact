// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterCommand extends CommandBase {
  /** Creates a new ManualShooterCommand. */
  private ShooterSubsystem shooter;
  private final IndexerSubsystem indexerSubsystem;
  public ManualShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.indexerSubsystem = indexer;
    SmartDashboard.putNumber("Top shooter wheel percentage", 0);
    SmartDashboard.putNumber("Bottom shooter wheel percentage", 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topPercent = SmartDashboard.getNumber("Top shooter wheel percentage", 0);
    double bottomPercent = SmartDashboard.getNumber("Bottom shooter wheel percentage", 0);
    shooter.runByPercentage(bottomPercent, topPercent);
    indexerSubsystem.runFeeder();
    indexerSubsystem.runPreload();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runByPercentage(0, 0);
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
