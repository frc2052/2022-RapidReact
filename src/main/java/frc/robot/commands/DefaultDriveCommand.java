package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.DriveMode;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DashboardControlsSubsystem m_dashboardControlsSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

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

    protected double getTurnValue() {
        return m_rotationSupplier.getAsDouble();
    }

    @Override
    public void execute() {
        if(m_dashboardControlsSubsystem.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC) {    // Checks the drive mode selected in the SmartDashboard, isn't the most efficient to 
            m_drivetrainSubsystem.drive(                                                        // be checking each time, but there hasn't been any issues yet and should be just fine.
                ChassisSpeeds.fromFieldRelativeSpeeds(                                      // It also allows us to switch drive modes at any point during the match.
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    getTurnValue(),
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );
        } else {
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(), 
                    m_translationYSupplier.getAsDouble(), 
                    getTurnValue()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
