package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class manualExtend extends CommandBase {
    private final HookClimberSubsystem m_climbSubsystem;

    public manualExtend(HookClimberSubsystem subsystem) {
        m_climbSubsystem = subsystem;
        // Maybe add dependencies   
    }

    @Override
    public void execute() {
        m_climbSubsystem.extendArm();
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

