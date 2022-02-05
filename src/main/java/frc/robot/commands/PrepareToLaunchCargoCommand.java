// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;

public class PrepareToLaunchCargoCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final TwoWheelFlySubsystem twoWheelFly; 
  private final Intake intake;
  private final DigitalInput limitSwitch = new DigitalInput(0);

  public PrepareToLaunchCargoCommand(IndexerSubsystem indexer, TwoWheelFlySubsystem twoWheelFly, Intake intake) {
    this.indexer = indexer;
    this.twoWheelFly = twoWheelFly;
    this.intake = intake;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    twoWheelFly.runAtShootSpeed(Constants.ShooterSub.TOPWHEELSPEED, Constants.ShooterSub.BOTTOMWHEELSPEED);
    if (limitSwitch.get() == false) {
      indexer.runPreload();
      intake.hopperGo();
    } else {
      indexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    twoWheelFly.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
