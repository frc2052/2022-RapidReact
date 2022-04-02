package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to run the intake and make sure it's lowered.
 */
public class AutoIntakeCommand extends IntakeHopperRunCommand {
  private final IntakeSubsystem intake;

  public AutoIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(intake, indexer, hopper);
    this.intake = intake;

    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.armOut();
  }

  @Override
  public boolean isFinished(){
    return super.isFinished();
  }
}
