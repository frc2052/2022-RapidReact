package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;

public class RetractClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    private final BooleanSupplier overrideButtonPressed;
    private double retractPctOutput;

    public RetractClimberCommand(HookClimberSubsystem climber, BooleanSupplier overrideButtonPressed, double retractPctOutput) {
        this.climber = climber;
        this.overrideButtonPressed = overrideButtonPressed;
        this.retractPctOutput = retractPctOutput;

        addRequirements(this.climber);   
    }

    public RetractClimberCommand(HookClimberSubsystem climber, BooleanSupplier overrideButtonPressed) {
        this(climber, overrideButtonPressed, Constants.Climber.CLIMBER_RETRACT_SPEED_PCT);
    }

    @Override
    public void execute() {
        // Climber retracts.
        if (!climber.getIsLocked()) {
            climber.retract(retractPctOutput, overrideButtonPressed.getAsBoolean());
        } else {
            System.err.println("Climber locked!");
        }
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}


