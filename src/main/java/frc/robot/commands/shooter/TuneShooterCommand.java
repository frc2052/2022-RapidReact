// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooterCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;

  public TuneShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HopperSubsystem hopper) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.hopper = hopper;

    SmartDashboard.putNumber("Top shooter wheel speed TP100MS", 1000);
    SmartDashboard.putNumber("Bottom shooter wheel speed TP100MS", 1000);
    
    addRequirements(shooter, indexer, intake, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topSpeedTP100MS = SmartDashboard.getNumber("Top shooter wheel speed TP100MS", 0);
    double bottomSpeedTP100MS = SmartDashboard.getNumber("Bottom shooter wheel speed TP100MS", 0);
    shooter.shootAtSpeed(topSpeedTP100MS, bottomSpeedTP100MS);

    if (shooter.isAtSpeed()) {
      indexer.runFeeder();
      hopper.run();
      indexer.runPreload();
    } else {
      hopper.stop();
      indexer.stopFeeder();
      indexer.stopPreload();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shootAtPercentage(0, 0);
    indexer.stopFeeder();
    indexer.stopPreload();
    intake.stop();
    hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
