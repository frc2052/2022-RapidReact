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
        climberSubsystem.manualExtendArm();
    }

     @Override
    public void end(boolean interrupted) {
        climberSubsystem.manualStop();
    }
}

