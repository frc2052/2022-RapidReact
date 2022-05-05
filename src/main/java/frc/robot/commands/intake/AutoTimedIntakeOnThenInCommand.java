package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class AutoTimedIntakeOnThenInCommand extends IntakeHopperRunCommand {
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
  public AutoTimedIntakeOnThenInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, double deadlineSeconds) {
    super(intake, indexer, hopper);
    this.intake = intake;
    this.deadlineSeconds = deadlineSeconds;

    addRequirements(this.intake);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void initialize() {
    super.initialize();
    intake.armOut(); // Makes sure the intake arm is out when the command begins
  }

  @Override
  public void execute() {
      super.execute();
      //LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_INTAKE_ON);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(deadlineSeconds); // If the timer has reached or passed the deadlineSeconds, the command ends
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intake.armIn();
  }

}
