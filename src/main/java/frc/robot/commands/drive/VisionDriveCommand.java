package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDs.LEDChannel1;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.VisionCalculator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class VisionDriveCommand extends DefaultDriveCommand {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final TargetingSubsystem targeting;
    private double visionRotation;
    private boolean isLinedUp;

    public VisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision,
        ShooterSubsystem shooter
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
        this.shooter = shooter;
        this.targeting = TargetingSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        Rotation2d horizontalAngle = vision.getXRotation().minus(targeting.getDrivingHorizontalFiringOffsetAngleDegrees());      // Horizontal offset from the Limelight's crosshair to target + driving while shooting offset.
        double horizontalAngleDegrees = horizontalAngle.getDegrees();
        isLinedUp = false;

        if(vision.getHasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(horizontalAngleDegrees) > 4) {
//                visionRotation = -Math.copySign(Math.PI/2, horizontalAngle);
                visionRotation = -Math.copySign(Math.toRadians(horizontalAngleDegrees * 9) , horizontalAngleDegrees); // Dynamically changes rotation speed to be faster at a larger tx,
                targeting.setIsLinedUpToShoot(true);
            } else if(Math.abs(horizontalAngleDegrees) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, horizontalAngleDegrees);
                targeting.setIsLinedUpToShoot(true);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                targeting.setIsLinedUpToShoot(true);
                isLinedUp = true;
            }
        } else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            visionRotation = 0;
        }

        if (isLinedUp) {
            LEDChannel1.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_FOUND);
        } else {
            LEDChannel1.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGETING);
        }
        return visionRotation;
    }

    public boolean getIsLinedUp() {
        return isLinedUp;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
        LEDChannel1.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT);
    }
}
