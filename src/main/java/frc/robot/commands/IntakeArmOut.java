package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeArmOut extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param subsystem The subsystem used by this command.
   */
    public IntakeArmOut(IntakeSubsystem subsystem) {
      m_intakeSubsystem = subsystem;
      // Maybe add dependencies   
    }
      
    @Override
    public void execute() {
      m_intakeSubsystem.intakeArmOut();
      m_intakeSubsystem.intakeOn();
      m_intakeSubsystem.hopperGo();
      //arm extends and spins the wheels
    }
    @Override
    public boolean isFinished(){
      return true;
    }
  
}
