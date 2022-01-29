package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class ArmOut extends CommandBase {
    private final Intake m_intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param subsystem The subsystem used by this command.
   */
    public ArmOut(Intake subsystem) {
      m_intakeSubsystem = subsystem;
      // Maybe add dependencies   
    }
      
    @Override
    public void initialize() {
    m_intakeSubsystem.intakeArmOut();
    m_intakeSubsystem.intakeOn();

    }
    @Override
    public boolean isFinished(){
      return true;
    }
  
}
