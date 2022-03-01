// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WaitEncoderInitializationCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final Timer timer;
    private final int timeoutSeconds;

    private boolean success = false;

    public WaitEncoderInitializationCommand(DrivetrainSubsystem drivetrain) {
        this(drivetrain, 10);
    }

    /** Creates a new VerifyEncoderInitializationCommand. */
    public WaitEncoderInitializationCommand(DrivetrainSubsystem drivetrain, int timeoutSeconds) {
        this.drivetrain = drivetrain;

        this.timer = new Timer();
        this.timeoutSeconds = timeoutSeconds;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        success = drivetrain.encodersInitialized(10);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= timeoutSeconds || success;
    }
}
