package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ExtendClimberCommand extends CommandBase {
    private final HookClimberSubsystem climberSubsystem;

    public ExtendClimberCommand(HookClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(this.climberSubsystem);   
    }

    @Override
    public void execute() {
        // Climber extends. 
        if (!climberSubsystem.isLocked()) {
            climberSubsystem.manualExtendArm();
        } else {
            System.err.println("Climber locked!");
        }
    }

     @Override
    public void end(boolean interrupted) {
        climberSubsystem.manualStop();
    }
}

