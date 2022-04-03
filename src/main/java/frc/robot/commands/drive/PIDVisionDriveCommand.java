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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
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
            1, 0, 0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        pidController.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        Rotation2d horizontalAngle = vision.getXRotation().minus(targeting.getDrivingHorizontalFiringOffsetAngleDegrees());      // Horizontal offset from the Limelight's crosshair to target + driving while shooting offset.

        pidController.setTolerance(vision.getTolerance());

        return pidController.calculate(
            drivetrain.getGyroscopeRotation().getDegrees(),
            drivetrain.getGyroscopeRotation().plus(horizontalAngle).getDegrees()
            );
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
        LEDSubsystem.getInstance().clearStatusMode();
    }
}
