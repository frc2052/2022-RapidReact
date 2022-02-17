// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PrepareToLaunchCargoCommand extends HopperBaseCommand {
  private final VisionSubsystem visionSubsystem;

  public PrepareToLaunchCargoCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, VisionSubsystem visionSubsystem, HopperSubsystem hopperSubsystem) {
    super(shooterSubsystem, indexerSubsystem, hopperSubsystem);
    this.visionSubsystem = visionSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TESTING Mathmatical calculations for running shooter at required velocity
    // with Limelight's calculated distance.
    if (visionSubsystem.hasValidTarget()) {
      double distance = visionSubsystem.getXDistanceToUpperHub();
      double reqVelocity = shooterSubsystem.calculateReqProjectileVelocity(distance);
      double reqAngularVelocity = reqVelocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS;
      // double reqRPM = twoWheelFly.calculateReqShooterRPM(reqVelocity);

      shooterSubsystem.setBothWheelVelocities(reqAngularVelocity);
    }

    shooterSubsystem.runAtShootSpeed();
    if (!indexerSubsystem.getCargoStagedDetected()) {
      // Keep running all the wheels until all the balls are staged
      indexerSubsystem.runPreload();
      indexerSubsystem.runFeeder();
      hopperSubsystem.hopperGo();
    } else if (indexerSubsystem.getCargoStagedDetected() && !indexerSubsystem.getCargoPreStagedDetected()) {
      // The staged detector shows a ball ready to be fired but no second ball is
      // detected
      indexerSubsystem.stopFeeder();
      indexerSubsystem.runPreload();
      hopperSubsystem.hopperGo();
    } else {
      // Two balls are loaded and no more can be taken
      indexerSubsystem.stopFeeder();
      indexerSubsystem.stopPreload();
      hopperSubsystem.hopperStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopFeeder();
    indexerSubsystem.stopPreload();
    shooterSubsystem.stop();
    hopperSubsystem.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
