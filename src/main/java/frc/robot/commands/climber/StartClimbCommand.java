package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class StartClimbCommand extends CommandBase {
    private final HookClimberSubsystem climberSubsystem;

    public StartClimbCommand(HookClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void execute() {
        // Climb to 2nd bar height then shift climber into forward position
        climberSubsystem.setArmPostionInches(60.25);
        climberSubsystem.shiftClimberForward();
    }

    @Override
    public boolean isFinished(){
        return climberSubsystem.isAtDesiredPosition();
    }    
}
