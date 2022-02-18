package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final HopperSubsystem hopper;

    public IntakeStopCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopper) {
      this.intakeSubsystem = intakeSubsystem;
      this.hopper = hopper;

      addRequirements(intakeSubsystem, hopper);
    }
  
    @Override
    public void execute() {
        intakeSubsystem.intakeStop();
        hopper.hopperStop();
        //stops the intake
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
