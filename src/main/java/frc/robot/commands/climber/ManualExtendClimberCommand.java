package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;

public class ManualExtendClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    private double retractPctOutput;

    public ManualExtendClimberCommand(HookClimberSubsystem climber, double retractPctOutput) {
        this.climber = climber;
        this.retractPctOutput = retractPctOutput;

        addRequirements(this.climber);   
    }

    public ManualExtendClimberCommand(HookClimberSubsystem climber) {
        this(climber, Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT);
    }

    @Override
    public void execute() {
        climber.movePctOutput(retractPctOutput);
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}


