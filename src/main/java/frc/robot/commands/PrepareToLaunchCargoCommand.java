// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class PrepareToLaunchCargoCommand extends HopperBaseCommand {
  private final VisionSubsystem vision;

  public PrepareToLaunchCargoCommand(TwoWheelFlySubsystem twoWheelFly, IndexerSubsystem indexer, VisionSubsystem vision, HopperSubsystem hopper) {
    super(twoWheelFly, indexer, hopper);
    this.vision = vision;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TESTING Mathmatical calculations for running shooter at required velocity with Limelight's calculated distance.
    if(vision.hasValidTarget()) {
      double distance = vision.getXDistanceToUpperHub();
      double reqVelocity = twoWheelFly.calculateReqProjectileVelocity(distance);
      double reqAngularVelocity = reqVelocity/Constants.ShooterSub.FLYWHEEL_RADIUS_METERS;
      //double reqRPM = twoWheelFly.calculateReqShooterRPM(reqVelocity);

      twoWheelFly.setBothWheelVelocities(reqAngularVelocity);
    }

    twoWheelFly.runAtShootSpeed();
    if (!indexer.getCargoStagedDetected()) {
      //Keep running all the wheels until all the balls are staged
      indexer.runPreload();
      indexer.runFeeder();
      grassHopper.hopperGo();
    } else if (indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
      //The staged detector shows a ball ready to be fired but no second ball is detected
      indexer.stopFeeder();
      indexer.runPreload();
      grassHopper.hopperGo();
    } else {
      //Two balls are loaded and no more can be taken
      indexer.stopFeeder();
      indexer.stopPreload();
      grassHopper.hopperStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopFeeder();
    indexer.stopPreload();
    twoWheelFly.stop();
    grassHopper.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
