// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class PrepareToLaunchCargoCommand extends CommandBase {
  private final TwoWheelFlySubsystem twoWheelFly; 
  private final IndexerSubsystem indexer;
  private final VisionSubsystem vision;
  private final IntakeSubsystem intake;
  private final HopperSubsystem grassHopper;


  public PrepareToLaunchCargoCommand(TwoWheelFlySubsystem twoWheelFly, IndexerSubsystem indexer, VisionSubsystem vision, IntakeSubsystem intake, HopperSubsystem grassHopper) {
    this.twoWheelFly = twoWheelFly;
    this.indexer = indexer;
    this.vision = vision;
    this.intake = intake;
    this.grassHopper = grassHopper;
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
    if (indexer.getCargoPreStagedDetected() == false && indexer.getCargoStagedDetected() == false) {
      indexer.runPreload();
      grassHopper.hopperGo();
    } else if (indexer.getCargoPreStagedDetected() == false && indexer.getCargoStagedDetected() == true) {
        indexer.stopFeeder();
        indexer.runPreload();
    } else {
        indexer.stopFeeder();
        indexer.stopPreload();
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
