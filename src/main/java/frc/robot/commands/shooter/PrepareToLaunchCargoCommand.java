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
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.ProjectileCalculator;


public class PrepareToLaunchCargoCommand extends HopperBaseCommand {
  protected final ShooterSubsystem shooter;
  private final VisionSubsystem vision;

  public PrepareToLaunchCargoCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, VisionSubsystem vision) {
    super(indexer, hopper);
    this.shooter = shooter;
    this.vision = vision;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
      vision.setLED(LEDMode.ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    // TESTING Mathmatical calculations for running shooter at required velocity with Limelight's calculated distance.
    if(vision.hasValidTarget()) {
      double distance = vision.getXDistanceToUpperHub();
      double reqVelocity = ProjectileCalculator.calculateReqProjectileVelocity(distance);
      double reqAngularVelocity = reqVelocity/Constants.ShooterSub.FLYWHEEL_RADIUS_METERS;
      //double reqRPM = twoWheelFly.calculateReqShooterRPM(reqVelocity);

      // shooter.setBothWheelVelocities(reqAngularVelocity);
      // shooter.runAtShootSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.stop();
    vision.setLED(LEDMode.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
