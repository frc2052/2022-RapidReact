package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

import java.util.function.DoubleSupplier;

public class VisionDriveInRangeCommand extends VisionDriveCommand {
    private VisionSubsystem m_vision;
    private DrivetrainSubsystem m_driveTrain;
    private double verticalAngle;
    private double driveForwardRate;
    private boolean verticalIsLinedUp;

    public VisionDriveInRangeCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               VisionSubsystem vision,
                               DashboardControlsSubsystem dashboard) {
        super(drivetrainSubsystem,
        () -> { return 0.0; },
        translationYSupplier,
        vision,
        dashboard);

        this.m_vision = vision;
        this.m_driveTrain = drivetrainSubsystem;
    }

    @Override
    protected double getXSupplier() {
        m_vision.updateLimelight();
        verticalAngle = m_vision.getTy() + Constants.DriveTrain.kDrivingInRangeAngleOffsetDegrees;
        verticalIsLinedUp = false;

        // Don't know the actual rates I should be setting these to right now but this should work...

        if (m_vision.hasValidTarget()) {
            if (verticalAngle > 5) {
                driveForwardRate = 0.5;
            } else if (verticalAngle > 2) {
                driveForwardRate = 0.25;
            } else {
                driveForwardRate = 0;
                verticalIsLinedUp = true;
            }
        } else {
            driveForwardRate = 0.25;
        }

        return driveForwardRate;
    }

    public boolean getVerticalIsLinedUp() {
        return verticalIsLinedUp;
    }
}
