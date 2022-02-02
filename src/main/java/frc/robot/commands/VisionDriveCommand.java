package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.DriveMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

import java.util.function.DoubleSupplier;

public class VisionDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DashboardControlsSubsystem m_dashboardControlsSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    //private final DoubleSupplier m_rotationSupplier;
    private VisionSubsystem m_vision;

    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    public VisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               VisionSubsystem vision,
                               DashboardControlsSubsystem dashboard) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        m_dashboardControlsSubsystem = dashboard;
        this.m_vision = vision;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_vision.setLED(LEDMode.ON);
    }

    @Override
    public void execute() {
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

        if(m_dashboardControlsSubsystem.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC) {
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            visionRotation,
                            m_drivetrainSubsystem.getGyroscopeRotation()
            ));
        } else {
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(), 
                    m_translationYSupplier.getAsDouble(), 
                    visionRotation
            ));
        }
    }

    public boolean getIsLinedUp() {
        return isLinedUp;
    }

    @Override
    public void end(boolean interrupted) {
        m_vision.setLED(LEDMode.OFF);
        m_drivetrainSubsystem.stop();
    }
}
