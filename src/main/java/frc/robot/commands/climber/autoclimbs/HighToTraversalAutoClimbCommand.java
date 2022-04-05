package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.climber.ClimberArmsForwardCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class HighToTraversalAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    /**
     * Requires being already clipped onto Mid Bar correctly.
     * @param climber
     * @param drivetrain
     */
    public HighToTraversalAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        //170000
        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= 25000), // Slightly above bar so we can toggle arms back
            new ClimberArmsBackCommand(climber),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= 130000), // Just slightly below the bar
            new WaitUntilCommand(() ->  drivetrain.getIsTopOfSwing()),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> !climber.getIsBelowMaxHeight()),
            new ClimberArmsForwardCommand(climber)
            // new InstantCommand(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBING_TRAVERSAL))
            // new RetractClimberCommand(climber, () -> false, 0.8).withInterrupt(climber::getIsAtMinHeight)
        );
    }
}
