package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ManualExtendClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    private double extendPctOutput;

    public ManualExtendClimberCommand(HookClimberSubsystem climber, double extendPctOutput) {
        this.climber = climber;
        this.extendPctOutput = extendPctOutput;

        addRequirements(this.climber);   
    }

    public ManualExtendClimberCommand(HookClimberSubsystem climber) {
        this(climber, Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT);
    }

    @Override
    public void execute() {
        climber.movePctOutput(extendPctOutput);
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_EXTENDING);
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}


