package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.DriveMode;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DashboardControlsSubsystem dashboardControlsSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               DashboardControlsSubsystem dashboard) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        dashboardControlsSubsystem = dashboard;

        addRequirements(drivetrainSubsystem);
    }

    protected double getTurnValue() {
        return rotationSupplier.getAsDouble();
    }

    @Override
    public void execute() {
        if(dashboardControlsSubsystem.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC) {    // Checks the drive mode selected in the SmartDashboard, isn't the most efficient to 
            drivetrainSubsystem.drive(                                                        // be checking each time, but there hasn't been any issues yet and should be just fine.
                ChassisSpeeds.fromFieldRelativeSpeeds(                                      // It also allows us to switch drive modes at any point during the match.
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    getTurnValue(),
                    drivetrainSubsystem.getGyroscopeRotation()
                )
            );
        } else {
            drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    translationXSupplier.getAsDouble(), 
                    translationYSupplier.getAsDouble(), 
                    getTurnValue()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }
}
