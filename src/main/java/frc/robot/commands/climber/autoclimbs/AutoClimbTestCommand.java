package frc.robot.commands.climber.autoclimbs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.subsystems.HookClimberSubsystem;

public class AutoClimbTestCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    /**
     * Auto Climb testing command
     */
    public AutoClimbTestCommand(HookClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(climber::getIsAtMaxHeight),
            new RetractClimberCommand(climber, () -> true, 0.8).withInterrupt(() -> climber.getEncoderPosition() <= 5000)
        );
    }
}
