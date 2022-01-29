package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ArmIn extends CommandBase {
    private final Intake m_intakeSubsystem;

    public ArmIn(Intake subsystem) {
      m_intakeSubsystem = subsystem;
      // Maybe add dependencies   
  }
      
    @Override
    public void initialize() {
    m_intakeSubsystem.intakeArmIn();
    m_intakeSubsystem.intakeStop();

    }
    @Override
    public void execute() {
  
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
