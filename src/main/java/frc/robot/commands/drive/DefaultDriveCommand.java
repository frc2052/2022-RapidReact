package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.DriveMode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DashboardControlsSubsystem dashboardControls;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final BooleanSupplier tempFieldCentricButtonPressed;

    /**
     * The default drive command that runs when no other commands are, controlling the drivetrain.
     * @param drivetrain
     * @param translationXSupplier a DoubleSupplier that should pass the DriveJoystick's X
     * @param translationYSupplier a DoubleSupplier that should pass the DriveJoystick's Y
     * @param rotationSupplier a DoubleSupplier that should pass the TurnJoystick's value
     * @param dashboardControls DashboardControlsSubsystem instance for getting weather the drivemode should be field or robot centric
     */
    public DefaultDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier tempFieldCentricButtonPressed
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        this.dashboardControls = DashboardControlsSubsystem.getInstance();
        this.tempFieldCentricButtonPressed = tempFieldCentricButtonPressed;

        addRequirements(drivetrain);
    }

    protected double getTurnValue() { // Seperated as a method so it can be overriden by other commands if needed.
        return rotationSupplier.getAsDouble() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    @Override
    public void execute() {
        // Checks the drive mode selected in the SmartDashboard, isn't the most efficient to
        // be checking each time, but there hasn't been any issues yet and should be just fine.
        // It also allows us to switch drive modes at any point during the match.
        if(dashboardControls.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC || tempFieldCentricButtonPressed.getAsBoolean()) {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    translationYSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    getTurnValue(),
                    drivetrain.getGyroscopeRotation()
                )
            );
        } else {
            drivetrain.drive(
                new ChassisSpeeds(
                    translationXSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                    translationYSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
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
