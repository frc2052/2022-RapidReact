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
  private final ShooterSubsystem shooter;

  private Timer timer;
  private Timer timer2;
  private double deadlineSeconds;
  private boolean wasTwoBallsDetected;
  private boolean delayOverride;

  

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
    DrivetrainSubsystem drivetrain,
    double deadlineSeconds
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
    this.shooter = shooter;
    this.deadlineSeconds = deadlineSeconds;
  }

  public AutoShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    DrivetrainSubsystem drivetrain
  ) {
    this(
      shootMode,
      shooter,
      indexer,
      hopper,
      vision,
      drivetrain,
      0.5
    );
  }

  public AutoShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    DrivetrainSubsystem drivetrain,
    boolean delayOverride
  ) {
    this(
      shootMode,
      shooter,
      indexer,
      hopper,
      vision,
      drivetrain,
      0.5
    );    

    this.delayOverride = delayOverride;
  }

  @Override
  public void execute() {
      super.execute();
  }

  @Override
  public boolean isFinished() {
    if (!indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
      if (timer == null) { // This is the first time we've not seen a ball
          timer = new Timer();
          timer.start();
      }
      if (timer.get() >= deadlineSeconds) { // At least 1 sec has passed since a ball was last seen
        return true;
      }
    } else {
        clearTimer();   // Ball showed up, stop the timer
    }
    return false;
  }


  @Override
  public boolean isReadyToShoot() {
      if(shooter.isAtSpeed()) {
        if (timer2 == null) { // Creates timer if it hasn't started yet or was stopped.
            timer2 = new Timer();
            timer2.start();
        }

        if (timer2.hasElapsed(0.25)) {
            return super.isReadyToShoot();
        }
      } else {
        clearTimer();
      }

      return false;
  }

  /*
  @Override
  protected void IndexerExecute() {
    if (!delayOverride) {
        boolean stagedCargoDetected = indexer.getCargoStagedDetected();
        boolean preStagedCargoDetected = indexer.getCargoPreStagedDetected();

        if (!wasTwoBallsDetected) {
            if (preStagedCargoDetected) { // If the prestaged sensor detects a ball, sets boolean true because we'll need to slow it down.
                wasTwoBallsDetected = true;
            }
        }
        if (wasTwoBallsDetected && stagedCargoDetected && !preStagedCargoDetected && !delayOverride) { // If only staged beam break is broken and a second ball was on the way, stop the ball and check timer 
            indexer.stopFeeder();
            if (timer2 == null) { // Creates timer if it hasn't started yet or was stopped.
                timer2 = new Timer();
                timer2.start();
            }
            // System.err.println("TIME: " + timer2.get());
            if (timer2.hasElapsed(0.5)) { // If the beam's been broken for a quarter second, we can feed again.
                wasTwoBallsDetected = false;
                clearTimer();
            }
        } else {
            super.IndexerExecute();
        }
    } else {
        super.IndexerExecute();
    }
  }
  */

  private void clearTimer() {
      if(timer != null) {
          timer.stop();
          timer = null;
      }
  }
}
