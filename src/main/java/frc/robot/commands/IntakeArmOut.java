package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeArmOut extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
    public IntakeArmOut(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      // Maybe add dependencies   
    }
      
    @Override
    public void execute() {
      intakeSubsystem.intakeArmOut();
      intakeSubsystem.intakeOn();
      intakeSubsystem.hopperGo();
      //arm extends and spins the wheels
    }
    @Override
    public boolean isFinished(){
      return true;
    }
  
}
