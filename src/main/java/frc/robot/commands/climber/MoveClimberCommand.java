package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class MoveClimberCommand extends CommandBase {
    private final HookClimberSubsystem climber;
    protected double heightTP100MS;

    public MoveClimberCommand(HookClimberSubsystem climber, double heightTP100MS) {
        this.climber = climber;
        this.heightTP100MS = heightTP100MS;        
        addRequirements(this.climber);   
    }

    @Override
    public void execute() {
        climber.moveToHeight(heightTP100MS);
    }

     @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

