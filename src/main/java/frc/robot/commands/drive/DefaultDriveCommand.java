package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.DriveMode;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DashboardControlsSubsystem dashboardControls;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier,
        DashboardControlsSubsystem dashboardControls
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        this.dashboardControls = dashboardControls;

        addRequirements(drivetrain);
    }

    protected double getTurnValue() {
        return rotationSupplier.getAsDouble();
    }

    @Override
    public void execute() {
        // Checks the drive mode selected in the SmartDashboard, isn't the most efficient to
        // be checking each time, but there hasn't been any issues yet and should be just fine.
        // It also allows us to switch drive modes at any point during the match.
        if(dashboardControls.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC) {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    getTurnValue(),
                    drivetrain.getGyroscopeRotation()
                )
            );
        } else {
            drivetrain.drive(
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
        drivetrain.stop();
    }
}
