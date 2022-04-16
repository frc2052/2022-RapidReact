package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class RetractToMinClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;

    public RetractToMinClimberCommand(HookClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        double currentHeightTicks = climber.getEncoderPosition();

        if (currentHeightTicks <= Constants.Climber.MIN_CLIMBER_HEIGHT_TICKS) {
            climber.stop();
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_MAX_EXTENSION);
        } else if (currentHeightTicks <= Constants.Climber.MIN_CLIMBER_HEIGHT_TICKS + 5000) {
            climber.movePctOutput(Constants.Climber.CLIMBER_RETRACT_SPEED_PCT * 0.5);
        } else {
            climber.movePctOutput(Constants.Climber.CLIMBER_RETRACT_SPEED_PCT);
        }
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

