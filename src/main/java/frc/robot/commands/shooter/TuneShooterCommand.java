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
  private final HopperSubsystem grassHopper;

  public TuneShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HopperSubsystem grassHopper) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.grassHopper = grassHopper;

    SmartDashboard.putNumber("Top shooter wheel percentage", 0.45);
    SmartDashboard.putNumber("Bottom shooter wheel percentage", 0.45);
    
    addRequirements(shooter, indexer, intake, grassHopper);
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
    indexer.runFeeder();
    indexer.runPreload();
    intake.intakeArmOut();
    intake.intakeOn();
    grassHopper.hopperGo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runByPercentage(0, 0);
    indexer.stopFeeder();
    indexer.stopPreload();
    intake.intakeStop();
    grassHopper.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
