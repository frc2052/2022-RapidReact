package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ExtendClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    private final BooleanSupplier overrideButtonPressed;
    private double heightTP100MS;

    public ExtendClimberCommand(HookClimberSubsystem climber, double heightTP100MS) {
        this.climber = climber;
        this.overrideButtonPressed = overrideButtonPressed;
        this.heightTP100MS = heightTP100MS;        
        addRequirements(this.climber);   
    }

    @Override
    public void execute() {
        if (overrideButtonPressed.getAsBoolean()) {
            climber.movePctOutput(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT * 0.5);
        } else {
            climber.moveToHeight(heightTP100MS);
        }
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

