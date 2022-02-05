package frc.robot.commands;

// Command for simply rotating the robot to aim at the horizontal center of the upper hub. Similar to VisionDriveCommand.

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class VisionTurnInPlaceCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_vision;

    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    public VisionTurnInPlaceCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem vision) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_vision = vision;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_vision.setLED(LEDMode.ON);
    }

    @Override
    public void execute() {
        m_vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
        double rotation = m_vision.getRotationToTarget();
        isLinedUp = false;

        if(m_vision.hasValidTarget() || rotation == 0) { // Logic to set the chassis rotation speed based on horizontal offset.
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    rotation,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );
            visionRotation = m_vision.getRotationToTarget();
        } else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            m_drivetrainSubsystem.stop();
            isLinedUp = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_vision.setLED(LEDMode.OFF);
        m_drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isLinedUp;
    }
}
