// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;


public class PrepareToLaunchCargoCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final TwoWheelFlySubsystem twoWheelFly; 
  private final IntakeSubsystem intake;
  private final HopperSubsystem grassHopper;


  public PrepareToLaunchCargoCommand(IndexerSubsystem indexer, TwoWheelFlySubsystem twoWheelFly, IntakeSubsystem intake, HopperSubsystem grassHopper) {
    this.indexer = indexer;
    this.twoWheelFly = twoWheelFly;
    this.intake = intake;
    this.grassHopper = grassHopper;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    twoWheelFly.runAtShootSpeed();
    if (indexer.getCargoPreStagedDetected() == false && indexer.getCargoStagedDetected() == false) {
      indexer.runPreload();
      grassHopper.hopperGo();
    } else if (indexer.getCargoPreStagedDetected() == false && indexer.getCargoStagedDetected() == true) {
        indexer.stopFeeder();
        indexer.runPreload();
    } else {
        indexer.stopFeeder();
        indexer.stopPreload();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopFeeder();
    indexer.stopPreload();
    twoWheelFly.stop();
    grassHopper.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
