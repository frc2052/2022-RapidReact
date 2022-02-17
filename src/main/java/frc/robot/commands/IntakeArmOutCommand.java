package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeArmOutCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final HopperSubsystem grassHopper;

  /**
   * Creates a new ArmToggle.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
    public IntakeArmOutCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem grassHopper) {
      this.intakeSubsystem = intakeSubsystem;
      this.grassHopper = grassHopper;
    }
      
    @Override
    
    //arm extends and spins the wheels
    public void execute() {
      intakeSubsystem.intakeArmOut();
      intakeSubsystem.intakeOn();
      grassHopper.hopperGo();
    }

    @Override

    public boolean isFinished(){
      return true;
    }
  
}
