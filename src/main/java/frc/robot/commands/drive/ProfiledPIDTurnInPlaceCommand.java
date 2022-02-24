// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ProfiledPIDTurnInPlaceCommand extends ProfiledPIDCommand {
    private final DrivetrainSubsystem drivetrain;

    public ProfiledPIDTurnInPlaceCommand(
        DrivetrainSubsystem drivetrain, 
        Rotation2d originalGyroAngle, 
        Supplier<Rotation2d> deltaAngleSupplier
    ) {
        this(
            drivetrain,
            // The default ProfiledPIDController used by the command
            new ProfiledPIDController(
                // The PID gains
                5.0, // Kp
                0.0, // Ki
                0.0, // Kd
                // The motion profile constraints
                new TrapezoidProfile.Constraints(Math.PI, 2 * Math.PI)
            ),
            originalGyroAngle,
            deltaAngleSupplier
        );
    }

    public ProfiledPIDTurnInPlaceCommand(
        DrivetrainSubsystem drivetrain, 
        ProfiledPIDController profiledPIDController, 
        Rotation2d originalGyroAngle, 
        Supplier<Rotation2d> deltaAngleSupplier
    ) {
        super(
            // The ProfiledPIDController used by the command
            profiledPIDController,
            // This should return the measurement
            () -> drivetrain.getPose().getRotation().getRadians(),
            // This should return the goal (can also be a constant)
            deltaAngleSupplier.get().plus(originalGyroAngle).getRadians(),
            // This uses the output
            (output, setpoint) -> {
                // Use the output (and setpoint, if desired) here
                drivetrain.drive(new ChassisSpeeds(0.0, 0.0, output));
            },
            drivetrain
        );
        this.drivetrain = drivetrain;

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-Math.PI, Math.PI);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(Units.degreesToRadians(5), Units.degreesToRadians(10));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when isFinished() returns true.
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
