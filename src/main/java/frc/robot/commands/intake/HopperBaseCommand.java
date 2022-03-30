// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

/**
 * Class for the base command to run and control the hopper, being the indexer blue and green wheels and hopper omni wheels 
 */
public class HopperBaseCommand extends CommandBase {
  protected final IndexerSubsystem indexer;
  protected final HopperSubsystem hopper;
  protected final ShooterSubsystem shooter;

  private boolean preStagedIsAllianceColor;
  private boolean stagedIsAllianceColor;
  
  /**
   * @param indexer
   * @param hopper
   */
  public HopperBaseCommand(IndexerSubsystem indexer, HopperSubsystem hopper, ShooterSubsystem shooter) {
    this.indexer = indexer;
    this.hopper = hopper;
    this.shooter = shooter;

    addRequirements(indexer, hopper, shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.err.println("********************************" + indexerSubsystem.getCargoStagedDetected() + " ** " + indexerSubsystem.getCargoPreStagedDetected());

    if((preStagedIsAllianceColor || indexer.getCargoPreStagedDetected()) && (stagedIsAllianceColor || indexer.getCargoStagedDetected())) {
      if (!indexer.getCargoStagedDetected()) {
        //Keep running all the wheels until all the balls are staged
        indexer.runPreload();
        indexer.runFeeder();
        hopper.run();
      } else if (indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
        //The staged detector shows a ball ready to be fired but no second ball is detected
        indexer.stopFeeder();
        indexer.runPreload();
        hopper.run();
      } else {
        //Two balls are loaded and no more can be taken
        indexer.stopFeeder();
        indexer.stopPreload();
        hopper.stop();
      }
    } else if (!preStagedIsAllianceColor) {
      shooter.setShootAngle1();
      shooter.shootAtPercentage(0.1, 0.1);
      indexer.runFeeder();
    } else if (!stagedIsAllianceColor) {
      indexer.runPreloadReverse();
      hopper.runReverse();
    }

    // if (!preStagedIsAllianceColor) {
    //   CommandScheduler.getInstance().schedule(new NonVisionShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, FiringAngle.ANGLE_1, 2000, 2000));
    // }
  }

  // Returns true when the command should end, called every time execute is.
  @Override
  public boolean isFinished() {
    return indexer.getCargoStagedDetected() && indexer.getCargoPreStagedDetected();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopFeeder();
    indexer.stopPreload();
    hopper.stop();
  }
}
