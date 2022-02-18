// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ProjectileCalculator;

public class FeedTwoCargoLaunchCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final VisionSubsystem vision;

  public FeedTwoCargoLaunchCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, VisionSubsystem vision) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.vision = vision;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = vision.getXDistanceToUpperHub();
    double velocity = ProjectileCalculator.calculateReqProjectileVelocity(distance) + Constants.ShooterSub.CALCULATED_VELOCITY_OFFSET_MPS;
    double angularVelocity = velocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS;

    shooterSubsystem.setBothWheelVelocities(angularVelocity);

    shooterSubsystem.runAtShootSpeed();
    if (shooterSubsystem.isAtSpeed() && Math.abs(vision.getTx()) < 2) {
      indexerSubsystem.runFeeder();
      indexerSubsystem.runPreload();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
