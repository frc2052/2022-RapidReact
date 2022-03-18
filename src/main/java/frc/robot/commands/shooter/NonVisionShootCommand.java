// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NonVisionShootCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private final NonVisionShootMode nonVisionShootMode;

  private double topWheelVelocity;
  private double bottomWheelVelocity;

  public NonVisionShootCommand(NonVisionShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper,/* FiringAngle firingAngleMode,*/ double topWheelVelocity, double bottomWheelVelocity) {
    this.nonVisionShootMode = shootMode;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.topWheelVelocity = topWheelVelocity;
    this.bottomWheelVelocity = bottomWheelVelocity;

    addRequirements(shooter, indexer);
  }

  // @Override
  // public void initialize() {
  //     if (firingAngleMode == FiringAngle.ANGLE_1) {
  //       shooter.set
  //     }
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.shootAtSpeed(topWheelVelocity, bottomWheelVelocity);

    if (shooter.isAtSpeed()) {
      indexer.runFeeder();
      hopper.run();
      if (nonVisionShootMode == NonVisionShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stopFeeder();
    indexer.stopPreload();
    hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum NonVisionShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
