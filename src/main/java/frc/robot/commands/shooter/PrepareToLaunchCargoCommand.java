// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class PrepareToLaunchCargoCommand extends HopperBaseCommand {
  private final VisionSubsystem visionSubsystem;
  protected final ShooterSubsystem shooterSubsystem; 

  public PrepareToLaunchCargoCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, VisionSubsystem visionSubsystem, HopperSubsystem hopperSubsystem) {
    super(indexerSubsystem, hopperSubsystem);
    this.visionSubsystem = visionSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    // TESTING Mathmatical calculations for running shooter at required velocity with Limelight's calculated distance.
    if(visionSubsystem.hasValidTarget()) {
      double distance = visionSubsystem.getXDistanceToUpperHub();
      double reqVelocity = shooterSubsystem.calculateReqProjectileVelocity(distance);
      double reqAngularVelocity = reqVelocity/Constants.ShooterSub.FLYWHEEL_RADIUS_METERS;
      //double reqRPM = twoWheelFly.calculateReqShooterRPM(reqVelocity);

      shooterSubsystem.setBothWheelVelocities(reqAngularVelocity);
      shooterSubsystem.runAtShootSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
