package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final HopperSubsystem grassHopper;

    public IntakeStopCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem grassHopper) {
      this.intakeSubsystem = intakeSubsystem;
      this.grassHopper = grassHopper;
      // Maybe add dependencies   
  }
  
    @Override
    public void execute() {
        intakeSubsystem.intakeStop();
        grassHopper.hopperStop();
        //stops the intake
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}