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
  protected boolean isLinedUp;
  protected boolean delayOverride;

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

    SmartDashboard.putBoolean("Is Indexing Lined UP", isLinedUp);

    if (!wasTwoBallsDetected) {
      if (preStagedCargoDetected && stagedCargoDetected) { // If the prestaged sensor detects a ball, sets boolean true because we'll need to slow it down.
        wasTwoBallsDetected = true;
      }
    }

    if (wasTwoBallsDetected && stagedCargoDetected && !preStagedCargoDetected && !delayOverride) { // If only staged beam break is broken and a second ball was on the way, stop the ball and check timer 
      indexer.stopFeeder();
      if (timer == null) { // Creates timer if it hasn't started yet or was stopped.
          timer = new Timer();
          timer.start();
      }
      //System.err.println("Delaying Shooting for" + timer.get());
      if (timer.get() >= 0.5) { // If the beam's been broken for a quarter second, we can feed again.
        wasTwoBallsDetected = false;
        clearTimer();
      }
    } else if (shooter.isAtSpeed() && isLinedUp) {
        //System.err.println("Trying to Shoot 1st Ball");
      indexer.runFeeder();
      hopper.run();
      if (shootMode == ShootMode.SHOOT_ALL) {
        // System.err.println("Trying to Shoot 2nd Ball");
        indexer.runPreload();
      }
      LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.SHOOTING);
    } else {
        //System.err.println("Stopping Indexing");
      hopper.stop();
      indexer.stopFeeder();
      indexer.stopPreload();
    }
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

  private void clearTimer() {
    if(timer != null) {
        timer.stop();
        timer = null;
    }
  }

  public enum ShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
