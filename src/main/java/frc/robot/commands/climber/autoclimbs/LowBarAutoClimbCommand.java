package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.subsystems.HookClimberSubsystem;

public class LowBarAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    public LowBarAutoClimbCommand(HookClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() <= 20000),
            new RetractClimberCommand(climber, () -> true, 0.8).withInterrupt(() -> climber.getEncoderPosition() <= -2000)
        );
    }
}
