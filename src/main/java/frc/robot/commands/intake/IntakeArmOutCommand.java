package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to lower the intake.
 */
public class IntakeArmOutCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  public IntakeArmOutCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(indexer, hopper);
    this.intake = intake;

    addRequirements(this.intake);
  }
      
  @Override
  public void execute() {
    super.execute(); // Runs the hopper execute

    // Extend the intake arm and stops the intake wheels.
    intake.armOut();
    intake.run(); // TODO: Remove this line!
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intake.stop(); // TODO: Remove this line!
  }

  @Override
  public boolean isFinished(){
    return super.isFinished();
  }
}
