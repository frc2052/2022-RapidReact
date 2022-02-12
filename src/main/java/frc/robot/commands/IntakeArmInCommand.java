package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeArmInCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      // Maybe add dependencies   
  }
      
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        intakeSubsystem.intakeArmIn();
        intakeSubsystem.intakeStop();
        intakeSubsystem.hopperStop();
        //arm retracts and stops
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
