package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to lower the intake.
 */
public class IntakeArmOutAndRunCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;

  public IntakeArmOutAndRunCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(indexer, hopper);
    this.intake = intake;

    addRequirements(this.intake);
  }
      
  @Override
  public void execute() {
    super.execute(); // Runs the HopperBaseCommand execute method

    // Extend the intake arm and stops the intake wheels.
    intake.armOut();
    intake.run();
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intake.stop();
  }

  @Override
  public boolean isFinished(){
    return super.isFinished();
  }
}
