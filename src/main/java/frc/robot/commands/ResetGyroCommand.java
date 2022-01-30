package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ResetGyroCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrain;

    public ResetGyroCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.zeroGyroscope();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
