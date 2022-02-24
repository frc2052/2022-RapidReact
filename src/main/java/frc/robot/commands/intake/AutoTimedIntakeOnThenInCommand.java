package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to lower the intake.
 */
public class AutoTimedIntakeOnThenInCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;
  private Timer timer;
  private double deadlineSeconds;

  public AutoTimedIntakeOnThenInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, double deadlineSeconds) {
    super(indexer, hopper);
    this.intake = intake;
    this.deadlineSeconds = deadlineSeconds;

    addRequirements(this.intake);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void initialize() {
    intake.intakeArmOut();
  }
      
  @Override
  public void execute() {
    intake.intakeOn();
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intake.intakeStop();
    intake.intakeArmIn();
  }

  @Override
  public boolean isFinished() {
    if (timer == null) { // This is the first time we've not seen a ball
        timer = new Timer();
        timer.start();
    }
    if (timer.get() >= deadlineSeconds) { // At least 1 sec has passed since a ball was last seen
      return true;
    }
    return false;
  }
}
