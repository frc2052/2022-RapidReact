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
  private final IndexerSubsystem indexer;

  private Timer timer;

  /**
   * Command that extends ShootCommand to be used in Auto.
   * Finishes if the neither the preStaged or staged cargo sensors have seen a ball in 0.5 seconds.
   * @param shootMode
   * @param shooter
   * @param indexer
   * @param hopper
   * @param vision
   * @param drivetrain
   */
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

    this.indexer = indexer;
  }

  @Override
  public boolean isFinished() {
    if (!indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
      if (timer == null) { // This is the first time we've not seen a ball
          timer = new Timer();
          timer.start();
      }
      if (timer.get() >= 0.5) { // At least 1 sec has passed since a ball was last seen
        return true;
      }
    } else {
        clearTimer();   // Ball showed up, stop the timer
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
