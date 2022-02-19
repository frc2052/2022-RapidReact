// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final VisionSubsystem vision;
  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private final ShootMode shootMode;

  private final VisionCalculator visionCalculator;

  public ShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, VisionSubsystem vision) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.shootMode = shootMode;

    visionCalculator = new VisionCalculator();

    addRequirements(shooter, indexer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int distanceInches = visionCalculator.getDistanceInches(vision.getTy());
    ShooterDistanceConfig shooterConfig = visionCalculator.getShooterConfig(distanceInches);

    shooter.runAtShootSpeed(
      shooterConfig.getTopMotorVelocityTicksPerSecond(),
      shooterConfig.getBottomMotorVelocityTicksPerSecond()
    );

    if (shooter.isAtSpeed()) {
      indexer.runFeeder();
      if (shootMode == ShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopFeeder();
    indexer.stopPreload();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
