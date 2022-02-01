package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.driveMode;

import java.util.function.DoubleSupplier;

public class VisionDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DashboardControlsSubsystem m_dashboardControlsSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    //private final DoubleSupplier m_rotationSupplier;
    private VisionSubsystem m_vision;

    double visionRotation = 0;

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
    public void execute() {
        m_vision.updateLimelight();

        if(m_vision.hasValidTarget()) {
            if(Math.abs(m_vision.getTx()) > 5)
                visionRotation = -Math.copySign(Math.toRadians(m_vision.getTx() * 9) , m_vision.getTx());
            else if(Math.abs(m_vision.getTx()) > 2)
                visionRotation = -Math.copySign(Math.PI /4, m_vision.getTx());
            else
                visionRotation = 0; // must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
        } else
            visionRotation = 0;
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
        
        // System.out.println(visionRotation);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_dashboardControlsSubsystem.getSelectedDriveMode() == driveMode.FIELDCENTRIC) {
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

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
