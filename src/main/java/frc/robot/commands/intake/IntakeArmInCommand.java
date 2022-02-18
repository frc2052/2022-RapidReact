package frc.robot.commands.intake;

import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends HopperBaseCommand {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeArmInCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, IndexerSubsystem indexerSubsystem) {
      super(indexerSubsystem, hopperSubsystem);
      this.intakeSubsystem = intakeSubsystem;

      addRequirements(intakeSubsystem, hopperSubsystem);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intakeSubsystem.intakeArmIn();
        intakeSubsystem.intakeStop();
        //arm retracts and stops
    }
}
