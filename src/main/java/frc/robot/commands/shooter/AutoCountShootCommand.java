// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoCountShootCommand extends ShootCommand {
    private final IndexerSubsystem indexer;

  private Timer timer;
  private int ballsThroughStagedSensor;
  private boolean stagedCargoWasDetected;
  private boolean preStagedCargoWasDetected;
  private boolean lastSawStaged;
  private ShootMode shootMode;

  /**
   * Command that extends ShootCommand to be used in Auto. 
   * Counts balls that pass through staged cargo sensor and finishes if the amount determined by shootMode passes through.
   * Was tested and attempted but ended up unused in the 2022 season.
   * @param shootMode
   * @param shooter
   * @param indexer
   * @param hopper
   * @param vision
   * @param drivetrain
   */
  public AutoCountShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    DrivetrainSubsystem drivetrain
  ) {
    super(
      shootMode,
      shooter,
      indexer,
      hopper,
      vision,
      drivetrain
    );    

    this.indexer = indexer;
    this.shootMode = shootMode;
  }

  @Override
  public void initialize() {
      super.initialize();
      if (indexer.getCargoStagedDetected()) {
          stagedCargoWasDetected = true;
      }
  }

  // Lots of logic for different cases of counting balls through the staged sensor.
  @Override
  public boolean isFinished() {
    boolean stagedCargoDetected = indexer.getCargoStagedDetected();

    if (indexer.getCargoPreStagedDetected()) {
        preStagedCargoWasDetected = true;
    }

    // Checks if the delta of the reading was the ball leaving the sensor (was there and now is not)
    if (!stagedCargoDetected && lastSawStaged) {
        ballsThroughStagedSensor++;
    }
    lastSawStaged = stagedCargoDetected;

    if (shootMode == ShootMode.SHOOT_SINGLE && ballsThroughStagedSensor >= 1) {
        if (timer == null) {
          timer = new Timer();
          timer.start();
        }
        if (timer.hasElapsed(0.25)) {
            return true;
        }
    } else if (shootMode == ShootMode.SHOOT_ALL && stagedCargoWasDetected && preStagedCargoWasDetected && ballsThroughStagedSensor >= 2) {
        if (timer == null) {
            timer = new Timer();
            timer.start();
          }
        if (timer.hasElapsed(0.25)) {
            return true;
        }
    } else if (shootMode == ShootMode.SHOOT_ALL && stagedCargoWasDetected && !preStagedCargoWasDetected && ballsThroughStagedSensor >= 1) {
        if (timer == null) {
            timer = new Timer();
            timer.start();
          }
        if (timer.hasElapsed(0.25)) {
            return true;
        }
    }
    return false;
  }
}
