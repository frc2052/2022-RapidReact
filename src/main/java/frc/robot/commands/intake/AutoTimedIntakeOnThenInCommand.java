package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoTimedIntakeOnThenInCommand extends HopperBaseCommand {
  private final IntakeSubsystem intake;
  private Timer timer;
  private double deadlineSeconds;

  /**
   * Command used in auto to keep the intake on for a specified amount of time, then bring it back up.
   * @param intake
   * @param indexer
   * @param hopper
   * @param deadlineSeconds seconds that the intake will stay down
   */
  public AutoTimedIntakeOnThenInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, ShooterSubsystem shooter, double deadlineSeconds) {
    super(indexer, hopper, shooter);
    this.intake = intake;
    this.deadlineSeconds = deadlineSeconds;

    addRequirements(this.intake);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void initialize() {
    intake.armOut(); // Makes sure the intake arm is out when the command begins
  }
      
  @Override
  public void execute() {
    super.execute();
    intake.run();
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= deadlineSeconds) { // If the timer has reached or passed the deadlineSeconds, the command ends
      return true;
    }
    return false;
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intake.stop();
    intake.armIn();
  }

}
