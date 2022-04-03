package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ClimberArmsBackCommand extends CommandBase {
    private final HookClimberSubsystem climber;

  public ClimberArmsBackCommand(HookClimberSubsystem climber) {
      this.climber = climber;

      addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    climber.shiftBackward();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

