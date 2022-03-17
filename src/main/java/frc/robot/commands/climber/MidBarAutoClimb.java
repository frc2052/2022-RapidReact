package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HookClimberSubsystem;

public class MidBarAutoClimb extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    public MidBarAutoClimb(HookClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(climber::getIsAtMaxHeight),
            new RetractClimberCommand(climber, () -> true, 0.8).withInterrupt(() -> climber.getEncoderPosition() <= -2000)
        );
    }
}
