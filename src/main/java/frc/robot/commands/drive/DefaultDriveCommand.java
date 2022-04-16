package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
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

    // Slew rate limiters to make joystick inputs more gentle.
    // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

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

    @Override
    public void execute() {
        // Checks the drive mode selected in the SmartDashboard, isn't the most efficient to
        // be checking each time, but there hasn't been any issues yet and should be just fine.
        // It also allows us to switch drive modes at any point during the match.
        if(dashboardControls.getSelectedDriveMode() == DriveMode.FIELD_CENTRIC || tempFieldCentricButtonPressed.getAsBoolean()) {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    -modifyAxis(translationXSupplier.getAsDouble(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    -modifyAxis(translationYSupplier.getAsDouble(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    getTurnValue(),
                    drivetrain.getGyroscopeRotation()
                )
            );
        } else {
            drivetrain.drive(
                new ChassisSpeeds(
                    -modifyAxis(translationXSupplier.getAsDouble(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                    -modifyAxis(translationYSupplier.getAsDouble(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                    getTurnValue()
                )
            );
        }
    }

    // Seperated as a method so it may be overriden by other commands if needed.
    protected double getTurnValue() {
        return -modifyAxis(rotationSupplier.getAsDouble(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // This code borrowed from the SwerverDriveSpecialist Sample code
    private double modifyAxis(double value, SlewRateLimiter limiter) {
        // Deadband
        value = deadband(value, 0.05);
        // Square the axis for finer control at lower values
        value = limiter.calculate(Math.copySign(value * value, value));
        
        return value;
    }

    private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}
