package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final HopperSubsystem grassHopper;
    public IntakeArmInCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem grassHopper) {
      this.intakeSubsystem = intakeSubsystem;
      this.grassHopper = grassHopper;
      // Maybe add dependencies   
  }
      
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        intakeSubsystem.intakeArmIn();
        intakeSubsystem.intakeStop();
        grassHopper.hopperStop();
        //arm retracts and stops
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
