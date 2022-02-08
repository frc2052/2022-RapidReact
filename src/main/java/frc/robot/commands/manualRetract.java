package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class manualRetract extends CommandBase {
    private final HookClimberSubsystem m_climbSubsystem;

    public manualRetract(HookClimberSubsystem subsystem) {
        m_climbSubsystem = subsystem;
        // Maybe add dependencies   
    }

    @Override
    public void execute() {
        m_climbSubsystem.retractArm();
        //arm retracts 
    }
    @Override
    public boolean isFinished(){
        return false;
    }

     @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.armStop();
    }
}


