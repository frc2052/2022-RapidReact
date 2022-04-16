// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class OnlyIntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;

  /**
   * Command that only controls the intake in motor and nothing else. 
   * @param intake
   * @param indexer only for getting the cargo stage beam break sensors
   */
  public OnlyIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
    this.intake = intake;
    this.indexer = indexer;
    
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    intake.run();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
  
  @Override
  public boolean isFinished() {
    return !indexer.getCargoStagedDetected() || !indexer.getCargoPreStagedDetected();
  }
}
