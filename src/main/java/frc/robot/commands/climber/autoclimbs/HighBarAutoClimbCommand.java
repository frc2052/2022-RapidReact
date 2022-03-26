package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class HighBarAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    /**
     * Goes from Mid to High bar, requires being already clipped onto Mid Bar correctly.
     * @param climber
     * @param drivetrain
     */
    public HighBarAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new NextBarAutoClimbCommand(climber, drivetrain)
        );
    }
}
