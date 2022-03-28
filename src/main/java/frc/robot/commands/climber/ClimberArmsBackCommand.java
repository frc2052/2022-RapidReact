package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookClimberSubsystem;

public class ClimberArmsBackCommand extends CommandBase {
    private final HookClimberSubsystem climber;

  public ClimberArmsBackCommand(HookClimberSubsystem climber) {
      this.climber = climber;

      addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.shiftBackward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

