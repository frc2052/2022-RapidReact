package frc.robot.commands.climber.autoclimbs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.climber.ClimberArmsForwardCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class MidToHighAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    public MidToHighAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        //170000
        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= 25000), // Slightly above bar so we can toggle arms back
            new ClimberArmsBackCommand(climber),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> !climber.getIsBelowMaxHeight()),
            new ClimberArmsForwardCommand(climber)
            // new RetractClimberCommand(climber, () -> false, 0.8).withInterrupt(climber::getIsAtMinHeight)
        );
    }
}
