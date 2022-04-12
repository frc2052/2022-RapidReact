package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ManualRetractClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    private double retractPctOutput;

    public ManualRetractClimberCommand(HookClimberSubsystem climber, double retractPctOutput) {
        this.climber = climber;
        this.retractPctOutput = retractPctOutput;

        addRequirements(this.climber);   
    }

    public ManualRetractClimberCommand(HookClimberSubsystem climber) {
        this(climber, Constants.Climber.CLIMBER_RETRACT_SPEED_PCT);
    }

    @Override
    public void execute() {
        climber.movePctOutput(retractPctOutput);
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}


