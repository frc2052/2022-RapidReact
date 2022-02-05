package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;

public class PrepareToLauchCargoCommand extends CommandBase {
    
    private final IndexerSubsystem indexer;
    private final TwoWheelFlySubsystem twoWheelFly; 
    private final Intake intake;

  public PrepareToLaunchCargoCommand(IndexerSubsystem indexer, TwoWheelFlySubsystem twoWheelFly, Intake intake) {
      this.indexer = indexer;
      this.twoWheelFly = twoWheelFly;
      this.intake = intake;
  }

}
