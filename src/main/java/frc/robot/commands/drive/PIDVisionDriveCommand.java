package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class PIDVisionDriveCommand extends DefaultDriveCommand {

    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    // private final TargetingSubsystem targeting;
    private ProfiledPIDController pidController;

    /**
     * PID Controlled VisionDriveCommand for tracking the hub and by overriding the DefaultDriveCommand's getTurnValue method.
     * Currently less stable than old arbitrary value VisionDrive, was not further tuned because was a lower priority than
     * other tasks.
     * @param drivetrain
     * @param translationXSupplier
     * @param translationYSupplier
     * @param vision
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
        // this.targeting = TargetingSubsystem.getInstance();

        pidController = new ProfiledPIDController(
            7, 0, 0.3,//0.01, 0.01,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        // pidController.

        pidController.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        Rotation2d horizontalAngle = vision.getXRotation(); //vision.getXRotation().minus(targeting.getDrivingHorizontalFiringOffsetAngleDegrees());      // Horizontal offset from the Limelight's crosshair to target + driving while shooting offset.
        double rotation = 0;

        if (vision.getHasValidTarget()) {
            rotation = pidController.calculate(
                drivetrain.getOdometryRotation().getRadians(),
                drivetrain.getOdometryRotation().minus(horizontalAngle).getRadians()
            );
        }

        if (pidController.atSetpoint()) {
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_LINED_UP);
        } else {
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGETING);
        }

        // System.err.println("Vision angle: " + vision.getTx());
        // System.err.println("Gyro Rotation: " + drivetrain.getGyroscopeRotation().getDegrees());
        // System.err.println("PID Controller output: " + rotation);
        // System.err.println("PID Controller Position Error: " + pidController.getPositionError());

        // SmartDashboard.putNumber("Vision angle", vision.getTx()); //vision.getXRotation().getDegrees());
        // SmartDashboard.putNumber("Target Rotation", drivetrain.getOdometryRotation().minus(horizontalAngle).getRadians());
        // SmartDashboard.putNumber("PID Controller output", rotation);
        // SmartDashboard.putNumber("PID Controller Position Error", pidController.getPositionError());

        return rotation;
    }

    // @Override
    // public boolean isFinished() {
    //     return pidController.atSetpoint();
    // }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
        LEDSubsystem.getInstance().clearStatusMode();
    }
}
