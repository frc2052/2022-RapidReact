// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class ProfiledPIDVisionTurnInPlaceCommand extends ProfiledPIDTurnInPlaceCommand {
    private final VisionSubsystem vision;

    public ProfiledPIDVisionTurnInPlaceCommand(ProfiledPIDController profiledPIDController, DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(
            profiledPIDController,
            drivetrain, 
            () -> { return Rotation2d.fromDegrees(-vision.getTx()); }
        );
        this.vision = vision;
    }

    public ProfiledPIDVisionTurnInPlaceCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(
            drivetrain, 
            () -> { return Rotation2d.fromDegrees(-vision.getTx()); }
        );
        this.vision = vision;
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
    }
}
