package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class PIDVisionDriveCommand extends DefaultDriveCommand {

    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private PIDController pidController;

    /**
     * PID Controlled VisionDriveCommand for tracking the hub and by overriding the DefaultDriveCommand's getTurnValue method.
     */
    public PIDVisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision,
        DashboardControlsSubsystem dashboard
    ) {
        super(
            drivetrain,
            translationXSupplier,
            translationYSupplier,
            () -> { return 0.0; }, // This value will not be used because getTurn will be overriden.
            dashboard
        );
        
        this.vision = vision;
        this.drivetrain = drivetrain;

        pidController = new PIDController(1, 0, 0);
        pidController.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLED(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        pidController.setTolerance(0.07 * vision.getTy() + 3);

        return pidController.calculate(
            drivetrain.getGyroscopeRotation().getDegrees(),
            drivetrain.getGyroscopeRotation().getDegrees() + vision.getTx()
            );
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLED(LEDMode.OFF);
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT);
    }
}
