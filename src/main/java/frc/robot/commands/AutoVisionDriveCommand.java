package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class AutoVisionDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private VisionSubsystem m_vision;

    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    public AutoVisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem vision) {
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
        tx = m_vision.getTx();      // Horizontal offset from the Limelight's crosshair to target.
        isLinedUp = false;

        if(m_vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(tx) > 5) {
                visionRotation = -Math.copySign(Math.toRadians(m_vision.getTx() * 9) , tx);
            } else if(Math.abs(tx) > 2) {
                visionRotation = -Math.copySign(Math.PI /4, tx);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                isLinedUp = true;
            }
        } else
            visionRotation = 0;

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        visionRotation,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
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
