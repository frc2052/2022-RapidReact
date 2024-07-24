// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ShooterIndexingCommand extends Command {
  private final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;

  private ShootMode shootMode;
  protected boolean delayOverride = true;

  /** 
   * Command for indexing balls for the shooter in shoot commands.
   */
  public ShooterIndexingCommand(ShootMode shootMode, IndexerSubsystem indexer, HopperSubsystem hopper, ShooterSubsystem shooter) {
    this.shootMode = shootMode;
    this.indexer = indexer;
    this.hopper = hopper;
    this.shooter = shooter;

    addRequirements(this.indexer, this.hopper);
  }

  @Override
  public void execute() {
    IndexerExecute();
  }

  // Indexer logic - pulled into another method so it could be overridden selectively by higher commands that inherit from this one.
  protected void IndexerExecute() 
  {
    
    boolean stagedCargoDetected = indexer.getCargoStagedDetected();
    boolean preStagedCargoDetected = indexer.getCargoPreStagedDetected();

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

  public enum ShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
