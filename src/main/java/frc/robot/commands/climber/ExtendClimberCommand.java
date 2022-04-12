package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;

public class ExtendClimberCommand extends MoveClimberCommand {
    private final HookClimberSubsystem climber;

    public ExtendClimberCommand(HookClimberSubsystem climber) {
        super(climber, Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL);
        this.climber = climber; 
    }

    @Override
    public void execute() {
        if (climber.getIsVertical()) {
            super.heightTP100MS = Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL;
        } else {
            super.heightTP100MS = Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED;
        }

        super.execute();
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

