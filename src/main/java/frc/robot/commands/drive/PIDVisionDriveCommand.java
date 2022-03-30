package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDs.LEDChannel1;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class PIDVisionDriveCommand extends DefaultDriveCommand {

    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final TargetingSubsystem targeting;
    private ProfiledPIDController pidController;

    /**
     * PID Controlled VisionDriveCommand for tracking the hub and by overriding the DefaultDriveCommand's getTurnValue method.
     * Untested and probably unworking right now
     */
    public PIDVisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision
    ) {
        super(
            drivetrain,
            translationXSupplier,
            translationYSupplier,
            () -> { return 0.0; }, // This value will not be used because getTurn will be overriden.
            () -> { return false; }
        );
        
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targeting = TargetingSubsystem.getInstance();

        pidController = new ProfiledPIDController(
            0.1, 0.01, 0.01,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        pidController.enableContinuousInput(-180, 180);
        // pidController.
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        Rotation2d horizontalAngle = vision.getXRotation(); //vision.getXRotation().minus(targeting.getDrivingHorizontalFiringOffsetAngleDegrees());      // Horizontal offset from the Limelight's crosshair to target + driving while shooting offset.

        pidController.setTolerance(0.5);

        return pidController.calculate(
            drivetrain.getGyroscopeRotation().getDegrees(),
            drivetrain.getGyroscopeRotation().minus(horizontalAngle).getDegrees()
            );
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
        LEDChannel1.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT);
    }
}
