package frc.robot.commands.climber.autoclimbs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;

public class NextBarAutoClimbCommand extends SequentialCommandGroup{
    private final HookClimberSubsystem climber;
    
    /**
     * Requires being already clipped onto Mid Bar correctly.
     * @param climber
     * @param drivetrain
     */
    public NextBarAutoClimbCommand(HookClimberSubsystem climber, DrivetrainSubsystem drivetrain) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= 5000),
            new ClimberArmsBackCommand(climber),
            new ExtendClimberCommand(climber, () -> false).withInterrupt(() -> climber.getEncoderPosition() >= Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED - 5000),
            new WaitUntilCommand(() -> drivetrain.getGyroPitchDegrees() <= -10), // TEMP VALUE
            new ToggleClimberSolenoidCommand(climber),
            new RetractClimberCommand(climber, () -> false, 0.8).withInterrupt(climber::getIsAtMinHeight)
        );
    }
}
