package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeStop extends CommandBase {
    private final Intake m_intakeSubsystem;

    public IntakeStop(Intake subsystem) {
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
