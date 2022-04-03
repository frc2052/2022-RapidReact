package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ClimberArmsForwardCommand extends CommandBase {
    private final HookClimberSubsystem climber;

  public ClimberArmsForwardCommand(HookClimberSubsystem climber) {
      this.climber = climber;

      addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    climber.shiftForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

