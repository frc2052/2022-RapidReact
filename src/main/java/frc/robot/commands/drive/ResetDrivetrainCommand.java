// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    /** Comand intended to completely zero the drivetrain's odometry. */
    public ResetDrivetrainCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
