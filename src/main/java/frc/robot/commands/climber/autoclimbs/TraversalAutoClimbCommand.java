package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class TraversalAutoClimbCommand extends SequentialCommandGroup {
    private final HookClimberSubsystem climber;
 
    /**
     * Requires being already clipped onto Mid Bar correctly.
     * @param climber
     * @param drivetrain
     */
    public TraversalAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new NextBarAutoClimbCommand(climber, drivetrain),
            new NextBarAutoClimbCommand(climber, drivetrain)
        );
    }
}
