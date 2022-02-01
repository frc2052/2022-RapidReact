package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.driveMode;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DashboardControlsSubsystem m_dashboardControlsSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    //private double vxMetersPerSecond;
    //private double vyMetersPerSecond;
    //private double omegaRadiansPerSecond;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               DashboardControlsSubsystem dashboard) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        m_dashboardControlsSubsystem = dashboard;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        
        if(m_dashboardControlsSubsystem.getSelectedDriveMode() == driveMode.FIELDCENTRIC) {
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()
            ));
        } else {
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(), 
                    m_translationYSupplier.getAsDouble(), 
                    m_rotationSupplier.getAsDouble()
            ));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
