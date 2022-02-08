package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class StartClimbing extends CommandBase {
    private final HookClimberSubsystem m_climbSubsystem;

    public StartClimbing(HookClimberSubsystem subsystem) {
        m_climbSubsystem = subsystem;
        // Maybe add dependencies   
    }

    @Override
    public void execute() {
        m_climbSubsystem.setArmPostionInches(60.25);
        //arm retracts 
    }
    @Override
    public boolean isFinished(){
        return m_climbSubsystem.isAtDesiredPosition();
    }

    
}
