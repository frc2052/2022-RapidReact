package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final HopperSubsystem hopper;

    public IntakeStopCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
      this.intake = intake;
      this.hopper = hopper;

      addRequirements(intake, hopper);
    }
  
    @Override
    public void execute() {
        intake.intakeStop();
        hopper.hopperStop();
        // stops the intake
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
