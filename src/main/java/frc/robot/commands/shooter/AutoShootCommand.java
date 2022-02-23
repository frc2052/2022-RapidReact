// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

public class AutoShootCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  private final VisionSubsystem vision;
  private final AutoShootMode shootMode;

  private final VisionCalculator visionCalculator;

  private boolean isFinished = false;
  private Timer timer;

  public AutoShootCommand(
    AutoShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision
  ) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.vision = vision;
    this.shootMode = shootMode;

    visionCalculator = VisionCalculator.getInstance();

    SmartDashboard.putNumber("Shooter Velocity Boost Pct", 0);

    addRequirements(this.shooter, this.indexer, this.hopper);
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

    SmartDashboard.putNumber("Distance Inches", distanceInches);

    shooter.shootAtSpeed(
      shooterConfig.getTopMotorVelocityTicksPerSecond() * (shooterBoost + 1), // Boost the shooter velocity by a max of 110%
      shooterConfig.getBottomMotorVelocityTicksPerSecond() * (shooterBoost + 1)
    );

    if (shooter.isAtSpeed() && vision.isLinedUp()) {
      indexer.runFeeder();
      hopper.hopperGo();
      if (shootMode == AutoShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
    } else {
      hopper.hopperStop();
      indexer.stopFeeder();
      indexer.stopPreload();
    }

    if (!indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
        if (timer == null) { // This is the first time we've not seen a ball
            timer = new Timer();
            timer.start();
        }
        if (timer.get() >= 1) { // At least 1 sec has passed since a ball was last seen
            isFinished = true;
        }
    } else {
        clearTimer();   // Ball showed up, stop the timer
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter, feeder, and preloader.
    // Add delay here

    shooter.stop();
    hopper.hopperStop();
    indexer.stopFeeder();
    indexer.stopPreload();
    vision.setLED(LEDMode.OFF);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  private void clearTimer() {
      if(timer != null) {
          timer.stop();
          timer = null;
      }
  }

  public static enum AutoShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
