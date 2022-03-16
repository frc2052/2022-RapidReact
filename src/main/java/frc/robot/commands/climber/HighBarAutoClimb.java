package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HookClimberSubsystem;

public class MidBarAutoClimb extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    public HighBarAutoClimb(HookClimberSubsystem climber, BooleanSupplier overrideButtonIsPressed) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(climber::getIsAtMaxHeight),
            new RetractClimberCommand(climber, () -> true, 0.8).withInterrupt(() -> climber.getEncoderPosition() <= -2000),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= 5000),
            new ClimberArmsBackCommand(climber),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(climber::getIsAtMaxHeight),
            new ToggleClimberSolenoidCommand(climber),
            new RetractClimberCommand(climber, () -> false, 0.8).withInterrupt(climber::getIsAtMinHeight)
        );
    }
}
