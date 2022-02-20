// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsystem grassHopper;
  private final VisionSubsystem vision;
  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private final ShootMode shootMode;

  private final VisionCalculator visionCalculator;

  public ShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem grassHopper, 
    VisionSubsystem vision
  ) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.grassHopper = grassHopper;
    this.vision = vision;
    this.shootMode = shootMode;

    visionCalculator = new VisionCalculator();

    SmartDashboard.putNumber("Shooter Velocity Boost Pct", 0);

    addRequirements(this.shooter, this.indexer, this.grassHopper);
  }

  @Override
  public void initialize() {
      vision.setLED(LEDMode.ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int distanceInches = visionCalculator.getDistanceInches(vision.getTy());
    ShooterDistanceConfig shooterConfig = visionCalculator.getShooterConfig(distanceInches);

    double shooterBoost = SmartDashboard.getNumber("Shooter Velocity Boost Pct", 0);
    if (shooterBoost < 0) {
      shooterBoost = 0;
    } else if (shooterBoost > 0.1) {
      shooterBoost = 0.1;
    }

    System.err.println("LIMELIGHT DISTANCE INCHES: " + distanceInches);
    System.err.println("DISTANCE INCHES: " + shooterConfig.getDistanceInches());
    System.err.println("TOP MOTOR VELOCITY: " + shooterConfig.getTopMotorVelocityTicksPerSecond());
    System.err.println("BOTTOM MOTOR VELOCITY: " + shooterConfig.getBottomMotorVelocityTicksPerSecond());

    SmartDashboard.putNumber("Distance Inches", distanceInches);

    shooter.shootAtSpeed(
      shooterConfig.getTopMotorVelocityTicksPerSecond() * (shooterBoost + 1), // Boost the shooter velocity by a max of 110%
      shooterConfig.getBottomMotorVelocityTicksPerSecond() * (shooterBoost + 1)
    );

    if (shooter.isAtSpeed() && vision.isLinedUp()) {
      indexer.runFeeder();
      grassHopper.hopperGo();
      if (shootMode == ShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
    } else {
      grassHopper.hopperStop();
      indexer.stopFeeder();
      indexer.stopPreload();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    grassHopper.hopperStop();
    indexer.stopFeeder();
    indexer.stopPreload();
    vision.setLED(LEDMode.OFF);
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
