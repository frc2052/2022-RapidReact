// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

/**
 * Class for the base command to run and control the hopper, being the indexer blue and green wheels and hopper compliant wheels 
 */
public class HopperBaseCommand extends CommandBase {
  protected final IndexerSubsystem indexer;
  protected final HopperSubsystem hopper;
  
  /**
   * Base command for running and indexing balls into the hopper
   * @param indexer
   * @param hopper
   */
  public HopperBaseCommand(IndexerSubsystem indexer, HopperSubsystem hopper) {
    this.indexer = indexer;
    this.hopper = hopper;

    // Requirements commented out to be able to run the intake while shooting. The two still will not conflict because of our button mappings, but because this was here it was in a loop of interuppting the shoot command when it didn't need to.
    //addRequirements(indexer, hopper);
  }

  @Override
  public void execute() {
    if (!indexer.getCargoStagedDetected()) {  //Keep running all the wheels until all the balls are staged
      indexer.runPreload();
      indexer.runFeeder();
      hopper.run();
      LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.INTAKE_ON_0_BALLS);
    } else if (indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {  //The staged detector shows a ball ready to be fired but no second ball is detected
      indexer.stopFeeder();
      indexer.runPreload();
      hopper.run();
      // LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.INTAKE_ON_1_BALL); // LED changes for balls in hopper are done in IndexerSubsystem periodic
    } else {  //Two balls are loaded and no more can be intaken
      indexer.stopFeeder();
      indexer.stopPreload();
      hopper.stop();
      // LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.HOPPER_FULL);
    }
  }

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
