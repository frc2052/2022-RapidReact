package frc.robot.commands;

import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

import java.util.function.DoubleSupplier;

public class VisionDriveCommand extends DefaultDriveCommand {
    private VisionSubsystem m_vision;
    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    public VisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               VisionSubsystem vision,
                               DashboardControlsSubsystem dashboard) {
        super(drivetrainSubsystem,
        translationXSupplier,
        translationYSupplier,
        () -> { return 0.0; }, //this value will not be used because getTurnWillBeOverriden
        dashboard);

        this.m_vision = vision;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_vision.setLED(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        m_vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
        tx = m_vision.getTx();      // Horizontal offset from the Limelight's crosshair to target.
        isLinedUp = false;

        if(m_vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(tx) > 5) {
                visionRotation = -Math.copySign(Math.toRadians(m_vision.getTx() * 9) , tx); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(tx) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, tx);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                isLinedUp = true;
            }
        } else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            visionRotation = 0;
        }
        return visionRotation;
    }

    public boolean getIsLinedUp() {
        return isLinedUp;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_vision.setLED(LEDMode.OFF);
    }
}
