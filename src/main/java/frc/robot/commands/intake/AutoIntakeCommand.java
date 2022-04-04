package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

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
    super.initialize();
    intake.armOut();
  }

  @Override
  public void execute() {
      super.execute();
      LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_INTAKE_ON);
  }

  @Override
  public boolean isFinished(){
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
