package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class RetractClimberCommand extends CommandBase {
    private final HookClimberSubsystem climberSubsystem;
    private final BooleanSupplier overrideButtonPressed;

    public RetractClimberCommand(HookClimberSubsystem climberSubsystem, BooleanSupplier overrideButtonPressed) {
        this.climberSubsystem = climberSubsystem;
        this.overrideButtonPressed = overrideButtonPressed;

        addRequirements(this.climberSubsystem);   
    }

    @Override
    public void execute() {
        // Climber retracts.
        if (!climberSubsystem.getIsLocked()) {
            climberSubsystem.retract(overrideButtonPressed.getAsBoolean());
        } else {
            System.err.println("Climber locked!");
        }
    }

     @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}


