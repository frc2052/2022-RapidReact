package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ExtendToMaxClimberCommand extends MoveClimberCommand {
    private final HookClimberSubsystem climber;

    public ExtendToMaxClimberCommand(HookClimberSubsystem climber) {
        super(climber, Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL);
        this.climber = climber; 
    }

    @Override
    public void execute() {
        double maxHeight = climber.getIsVertical() ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED;
        double currentHeightTicks
        if (climber.getIsAboveMaxHeight()) {
            climber.stop();
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_MAX_EXTENSION);
        } else if (getIsBelowMaxHeight()) {
            climberMotor.set(ControlMode.PercentOutput, extendPctOutput);
        } else {
            // System.err.println("Climber extended to (or past) the max height!");
        }
        } else {
            climberMotor.set(ControlMode.MotionMagic, Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL);
        }

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

