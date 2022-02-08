package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeStop(IntakeSubsystem subsystem) {
      m_intakeSubsystem = subsystem;
      // Maybe add dependencies   
  }
  
    @Override
    public void execute() {
        m_intakeSubsystem.intakeStop();
        m_intakeSubsystem.hopperStop();
        //stops the intake
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
