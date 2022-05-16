package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ExtendToMaxClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;

    /**
     * Extends the climber to the max extension height to stay in frame perimeter,
     * which changes depending on if the arms are vertical or tilted back.
     * @param climber
     */
    public ExtendToMaxClimberCommand(HookClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        double maxHeight = climber.getIsVertical() ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED;
        double currentHeightTicks = climber.getEncoderPosition();

        if (currentHeightTicks >= maxHeight) {
            climber.stop();
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_MAX_EXTENSION);
        } else if (currentHeightTicks >= maxHeight * 0.95) {
            climber.movePctOutput(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT * 0.5);
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_EXTENDING);
        } else {
            climber.movePctOutput(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT);
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_EXTENDING);
        }
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

