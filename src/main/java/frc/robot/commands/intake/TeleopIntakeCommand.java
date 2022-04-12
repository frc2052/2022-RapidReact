package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in teleop to run the intake and make sure it's lowered, and puts it up when finished.
 */
public class TeleopIntakeCommand extends IntakeHopperRunCommand {
  private final IntakeSubsystem intake;

  public TeleopIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
    super(intake, indexer, hopper);
    this.intake = intake;
  }

  @Override
  public void initialize() {
    super.initialize();
    intake.armOut();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.armIn();
  }
}
