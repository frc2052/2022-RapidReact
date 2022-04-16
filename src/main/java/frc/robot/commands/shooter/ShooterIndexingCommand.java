// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ShooterIndexingCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;

  private ShootMode shootMode;
  private boolean wasTwoBallsDetected;
  private Timer timer;
  protected boolean delayOverride = true;

  /** 
   * Command for indexing balls for shooter in shoot commands.
   */
  public ShooterIndexingCommand(ShootMode shootMode, IndexerSubsystem indexer, HopperSubsystem hopper, ShooterSubsystem shooter) {
    this.shootMode = shootMode;
    this.indexer = indexer;
    this.hopper = hopper;
    this.shooter = shooter;

    addRequirements(this.indexer, this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    boolean stagedCargoDetected = indexer.getCargoStagedDetected();
    boolean preStagedCargoDetected = indexer.getCargoPreStagedDetected();

    // Old Logic for having a delay in stopping the preloaded ball to make it lose its momentum and solve shooter wheel speed issues
    // if (!wasTwoBallsDetected) {
    //   if (preStagedCargoDetected) { // If the prestaged sensor detects a ball, sets boolean true because we'll need to slow it down.
    //     wasTwoBallsDetected = true;
    //   }
    // }

    // if (wasTwoBallsDetected && stagedCargoDetected && !preStagedCargoDetected && !delayOverride) { // If only staged beam break is broken and a second ball was on the way, stop the ball and check timer 
    //   indexer.stopFeeder();
    //   if (timer == null) { // Creates timer if it hasn't started yet or was stopped.
    //       timer = new Timer();
    //       timer.start();
    //   }
    //   if (timer.get() >= 0.0) { // If the beam's been broken for a quarter second, we can feed again.
    //     wasTwoBallsDetected = false;
    //     clearTimer();
    //   }
    // } else 
    if (shootMode == ShootMode.SHOOT_ALL && !stagedCargoDetected && preStagedCargoDetected) {  //The staged detector shows a ball ready to be fired but no second ball is detected
      indexer.runFeeder();
      indexer.runPreload();
      hopper.run();
    } else if (isReadyToShoot()) {
      indexer.runFeeder();
      hopper.run();
      if (shootMode == ShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
      LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.SHOOTING);
    } else {
      indexer.stopFeeder();
      if (preStagedCargoDetected) {
        indexer.stopPreload();
        hopper.stop();
      } else {
        indexer.runPreload();
        hopper.run();
      }
    }
  }

  // Override this in different shoot commands for custom shoot control
  protected boolean isReadyToShoot() {
    return shooter.isAtSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
    indexer.stopFeeder();
    indexer.stopPreload();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // private void clearTimer() {
  //   if(timer != null) {
  //       timer.stop();
  //       timer = null;
  //   }
  // }

  public enum ShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
