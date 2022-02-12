// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;

public class PrepareToLaunchCargoCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final TwoWheelFlySubsystem twoWheelFly; 
  private final IntakeSubsystem intake;
  // private final DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitch.INDEXER_PRELOAD);
  // private final DigitalInput limitSwitchTwo = new DigitalInput(Constants.LimitSwitch.INDEXER_FEEDER);

  public PrepareToLaunchCargoCommand(IndexerSubsystem indexer, TwoWheelFlySubsystem twoWheelFly, IntakeSubsystem intake) {
    this.indexer = indexer;
    this.twoWheelFly = twoWheelFly;
    this.intake = intake;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    twoWheelFly.runAtShootSpeed();
    // if ( limitSwitch.get() == false ) {
    //   indexer.runPreload();
    //   intake.hopperGo();
    // } else {
    //     indexer.stop();
    //   if (limitSwitchTwo.get() == false) {
    //     indexer.runPreload();
    //     intake.hopperGo();
    //   } else {
    //     indexer.stop();
    // }
  //}
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    twoWheelFly.stop();
    intake.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
