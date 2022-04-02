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

public class AutoShootCommand extends ShootCommand {

  private Timer timer;
  private int ballsThroughStagedSensor;
  private boolean stagedCargoDetected;
  private boolean lastSawStaged;
  private ShootMode shootMode;

  public AutoShootCommand(
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

    this.shootMode = shootMode;
  }

  @Override
  public boolean isFinished() {
    // if (!super.indexer.getCargoStagedDetected() && !super.indexer.getCargoPreStagedDetected()) {
    //   if (timer == null) { // This is the first time we've not seen a ball
    //       timer = new Timer();
    //       timer.start();
    //   }
    //   if (timer.get() >= 0.5) { // At least 1 sec has passed since a ball was last seen
    //     return true;
    //   }
    // } else {
    //     clearTimer();   // Ball showed up, stop the timer
    // }
    // return false;

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
    } else if (shootMode == ShootMode.SHOOT_ALL && ballsThroughStagedSensor >= 2) {
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

  private void clearTimer() {
      if(timer != null) {
          timer.stop();
          timer = null;
      }
  }
}
