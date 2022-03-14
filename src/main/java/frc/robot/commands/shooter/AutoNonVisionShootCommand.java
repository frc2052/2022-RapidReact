// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNonVisionShootCommand extends NonVisionShootCommand {

  private Timer timer;

  public AutoNonVisionShootCommand(
    NonVisionShootMode nonVisionShootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer,
    HopperSubsystem hopper,
    double topWheelVelocity,
    double bottomWheelVelocity
  ) {
    super(
        nonVisionShootMode,
        shooter,
        indexer,
        hopper,
        topWheelVelocity,
        bottomWheelVelocity
    );    
  }

  @Override
  public boolean isFinished() {
    if (!super.indexer.getCargoStagedDetected() && !super.indexer.getCargoPreStagedDetected()) {
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
