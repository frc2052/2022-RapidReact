package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeArmOutCommand extends HopperBaseCommand {
  private final IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public IntakeArmOutCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, IndexerSubsystem indexerSubsystem) {
    super(indexerSubsystem, hopperSubsystem);
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem, hopperSubsystem);
    }
      
  @Override
  public void execute() {
    super.execute();
    intakeSubsystem.intakeArmOut();
    intakeSubsystem.intakeOn();
      //arm extends and spins the wheels
  }
  @Override
  public boolean isFinished(){
    return super.isFinished();
  }

  @Override 
  public void end(boolean interrupted){
    super.end(interrupted);
    intakeSubsystem.intakeStop();
  }

}
