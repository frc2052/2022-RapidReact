// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final IndexerSubsystem indexer;
  private OuttakeMode outtakeMode;

  /**
   * Reverses the intake, hopper, and the indexer wheels for 1 or 2 balls depending on the outtakeMode
   * @param outtakeMode - ONE_BALL, or ALL_BALLS
   * @param intake
   * @param hopper
   * @param indexer
   */
  public OuttakeCommand(OuttakeMode outtakeMode, IntakeSubsystem intake, HopperSubsystem hopper, IndexerSubsystem indexer) {
    this.intake = intake;
    this.hopper = hopper;
    this.indexer = indexer;
    this.outtakeMode = outtakeMode;

    addRequirements(this.intake, this.hopper, this.indexer);
  }
  
  @Override
  public void execute() {
    intake.reverse();
    hopper.reverse();
    indexer.runPreloadReverse();
    if (outtakeMode == OuttakeMode.ALL_BALLS) {
        indexer.runFeederReverse();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    hopper.stop();
    indexer.stopPreload();
    indexer.stopFeeder();
  }

  public enum OuttakeMode {
      ONE_BALL,
      ALL_BALLS
  }
}