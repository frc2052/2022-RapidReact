// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.util.vision.ShooterDistanceConfig;

public class AutoNonVisionShootCommand extends NonVisionShootCommand {
  private final IndexerSubsystem indexer;

  private Timer timer;
  private double deadlineSeconds;

  /**
   * Command that extends NonVisionShootCommand, and just adds having a timer for making the command end after
   * neither indexer beam break sensor sees a ball for a certain amount of time.
   * @param nonVisionShootMode
   * @param shooter
   * @param indexer
   * @param hopper
   * @param firingAngle
   * @param topWheelVelocity
   * @param bottomWheelVelocity
   */
  public AutoNonVisionShootCommand(
    ShootMode nonVisionShootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer,
    HopperSubsystem hopper,
    FiringAngle firingAngle,
    double topWheelVelocity,
    double bottomWheelVelocity,
    double deadlineSeconds
  ) {
    super(
        nonVisionShootMode,
        shooter,
        indexer,
        hopper,
        firingAngle,
        topWheelVelocity,
        bottomWheelVelocity
    );

    this.indexer = indexer;
    this.deadlineSeconds = deadlineSeconds;
  }

  public AutoNonVisionShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, FiringAngle firingAngleMode, ShooterDistanceConfig shooterDistanceConfig) {
    super(shootMode, shooter, indexer, hopper, firingAngleMode, shooterDistanceConfig);
    this.indexer = indexer;
    deadlineSeconds = 0.5;
  }

  public AutoNonVisionShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, double topWheelVelocity, double bottomWheelVelocity) { // TODO Probably remove this constructor and add firing angle specification to all other NonVisionShootCommands, was only added to not break the functionality of already existing ones.
    this(shootMode, shooter, indexer, hopper, FiringAngle.ANGLE_1, topWheelVelocity, bottomWheelVelocity, 0.5);
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

  public void setDeadlineSeconds(double deadlineSeconds) {
    this.deadlineSeconds = deadlineSeconds;
  }

  private void clearTimer() {
      if(timer != null) {
          timer.stop();
          timer = null;
      }
  }
}
