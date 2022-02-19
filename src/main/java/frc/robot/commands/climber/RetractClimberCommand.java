package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class RetractClimberCommand extends CommandBase {
    private final HookClimberSubsystem climberSubsystem;

    public RetractClimberCommand(HookClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(this.climberSubsystem);   
    }

    @Override
    public void execute() {
        // Climber retracts.
        if (!climberSubsystem.getIsLocked()) {
            climberSubsystem.manualRetractArm();
        } else {
            System.err.println("Climber locked!");
        }
    }

     @Override
    public void end(boolean interrupted) {
        climberSubsystem.manualStop();
    }
}


