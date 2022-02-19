package frc.robot.commands.intake;

import frc.robot.commands.HopperBaseCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends HopperBaseCommand {
    private final IntakeSubsystem intake;
    
    public IntakeArmInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
        super(indexer, hopper);
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Arm retracts and stops.
        intake.intakeArmIn();
        intake.intakeStop();
    }
}
