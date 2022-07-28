package frc.robot.commands.intake;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

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
    if(!isFinished()) {  // initialize usually runs before isFinished, but this way we can make sure not to swap the solenoid
        intake.armOut();
    }
  }

  @Override
  public void execute() {
      super.execute();
      LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.INTAKE_ON_0_BALLS);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.armIn();
  }
}
