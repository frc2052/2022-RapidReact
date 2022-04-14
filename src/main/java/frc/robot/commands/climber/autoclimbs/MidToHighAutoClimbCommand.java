package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.climber.ClimberArmsForwardCommand;
import frc.robot.commands.climber.ExtendToMaxClimberCommand;
import frc.robot.commands.climber.MoveClimberCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class MidToHighAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    /**
     * Command for automatically transferring robot from the Mid bar to the High bar, by extending the hooks just a bit,
     * puts the arms back, extends them to max height, and then shifts arms forward again.
     * Requires being already clipped onto Mid Bar correctly. 
     * @param climber
     * @param drivetrain
     */
    public MidToHighAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        //170000
        this.addCommands(
            new ExtendToMaxClimberCommand(climber).withInterrupt(() -> climber.getEncoderPosition() >= 25000), // Slightly above bar so we can toggle arms back
            new ClimberArmsBackCommand(climber),
            new ExtendToMaxClimberCommand(climber).withInterrupt(() -> climber.getIsAboveMaxHeight()),
            new ClimberArmsForwardCommand(climber)
            // new InstantCommand(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBING_HIGH_BAR))
            // new RetractClimberCommand(climber, () -> false, 0.8).withInterrupt(climber::getIsAtMinHeight)
        );
    }
}
