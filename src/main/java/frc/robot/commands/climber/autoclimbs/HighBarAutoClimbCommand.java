package frc.robot.commands.climber.autoclimbs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class HighBarAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    public HighBarAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new MidBarAutoClimbCommand(climber),
            new NextBarAutoClimbCommand(climber, drivetrain)
        );
    }
}
