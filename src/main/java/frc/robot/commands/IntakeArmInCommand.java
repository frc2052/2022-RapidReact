package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final HopperSubsystem grassHopper;
    public IntakeArmInCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem grassHopper) {
      this.intakeSubsystem = intakeSubsystem;
      this.grassHopper = grassHopper;
      // Maybe add dependencies idk that's what they did in 2020
  }
      
    @Override
    public void initialize() {
        
    }
    @Override
    
    //arm retracts and stops
    public void execute() {
        intakeSubsystem.intakeArmIn();
        intakeSubsystem.intakeStop();
        grassHopper.hopperStop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
  
}
