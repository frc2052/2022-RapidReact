package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;

public class MoveClimberToHeightCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    protected double heightTicks;
    private double currentHeightTicks;

    /**
     * Command to run the climber to specified height in ticks.
     * @param climber
     * @param heightTicks
     */
    public MoveClimberToHeightCommand(HookClimberSubsystem climber, double heightTicks) {
        this.climber = climber;
        this.heightTicks = heightTicks;        
        addRequirements(this.climber);   
    }

    @Override
    public void execute() {
        currentHeightTicks = climber.getEncoderPosition();

        if (currentHeightTicks > heightTicks + 2500) {
            if (currentHeightTicks <= heightTicks * 1.03) {
                climber.movePctOutput(Constants.Climber.CLIMBER_RETRACT_SPEED_PCT * 0.5);
            } else {
                climber.movePctOutput(Constants.Climber.CLIMBER_RETRACT_SPEED_PCT);
            }
        } else if (currentHeightTicks < heightTicks - 2500) {
            if (currentHeightTicks >= heightTicks - 15000) {
                climber.movePctOutput(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT * 0.5);
            } else {
                climber.movePctOutput(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT);
            }
        } else {
            climber.stop();
        }
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentHeightTicks - heightTicks) < 2500;
    }
}

