package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeArmIn extends CommandBase {
    private final Intake m_intakeSubsystem;

    public IntakeArmIn(Intake subsystem) {
      m_intakeSubsystem = subsystem;
      // Maybe add dependencies   
  }
      
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        m_intakeSubsystem.intakeArmIn();
        m_intakeSubsystem.intakeStop();
        m_intakeSubsystem.hopperStop();
        //arm retracts and stops
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
