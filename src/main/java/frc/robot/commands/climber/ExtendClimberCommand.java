package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ExtendClimberCommand extends CommandBase {
    private final HookClimberSubsystem climberSubsystem;
    private final BooleanSupplier overrideButtonPressed;

    public ExtendClimberCommand(HookClimberSubsystem climberSubsystem, BooleanSupplier overrideButtonPressed) {
        this.climberSubsystem = climberSubsystem;
        this.overrideButtonPressed = overrideButtonPressed;        
        addRequirements(this.climberSubsystem);   
    }

    @Override
    public void execute() {
        // Climber extends. 
        if (!climberSubsystem.getIsLocked()) {
            climberSubsystem.extend(overrideButtonPressed.getAsBoolean());
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_EXTENDING);

        } else {
            System.err.println("Climber locked!");
        }
    }

     @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}

