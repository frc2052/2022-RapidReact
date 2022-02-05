package frc.robot.commands;

// Simple command for resetting gyro using a button on the secondary pannel
// TODO use Runnable version of whenPressed button command to make the button simply run RobotContainer's resetGyro method.

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
