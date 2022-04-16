package frc.robot.commands.intake;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in teleop to run the intake and make sure it's lowered, and puts it up when finished.
 */
public class TeleopOnlyIntakeCommand extends OnlyIntakeCommand {
  private final IntakeSubsystem intake;

  public TeleopOnlyIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
    super(intake, indexer);
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
