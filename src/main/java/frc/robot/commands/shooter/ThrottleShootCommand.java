// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ThrottleShootCommand extends ShooterIndexingCommand {
  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  private final DoubleSupplier throttle;
  
  int convertedThrottle = 0;

  /**
   * Command for shooting balls using vision.
   * @param shootMode
   * @param shooter
   * @param indexer
   * @param hopper
   */
  public ThrottleShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    DoubleSupplier throttle
  ) {
    super (
      shootMode,
      indexer,
      hopper,
      shooter
    );

    this.shooter = shooter;
    this.indexer = indexer;
    this.throttle = throttle;

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = (convertThrottleToPower(throttle.getAsDouble()));
    System.out.println(speed);

    shooter.runAtSpeed(
      speed,
      speed
    );

    super.execute();
  }

  @Override
  public boolean isReadyToShoot() {
      return shooter.isAtSpeed();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter, feeder, and preloader, and hopper
    super.end(interrupted);
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public int convertThrottleToPower(double throttle){
    double throttlePercent = Math.abs(throttle - 1) / 2;

    // Remap the 0-100 range of throttle input to the shooter range of 2000 - 19000 pct of Angle 1 (low2 + (value - low1) * (high2 - low2) / (high1 - low1))
    Long convertedThrottle = Math.round(throttlePercent * 17000 + 2000);

    return convertedThrottle.intValue();
  }
}