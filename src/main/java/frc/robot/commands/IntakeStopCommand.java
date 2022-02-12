package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeStopCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      // Maybe add dependencies   
  }
  
    @Override
    public void execute() {
        intakeSubsystem.intakeStop();
        intakeSubsystem.hopperStop();
        //stops the intake
    }
    @Override
    public boolean isFinished(){
        return true;
    }
  
}
