package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class VisionDriveCommand extends DefaultDriveCommand {
    private final VisionSubsystem vision;
    // private final TargetingSubsystem targeting;
    private double visionRotation;

    /**
     * Command for making the robot aim at the hub and track it while driving if desired.
     * Uses relatively arbitrary values but it worked and did so better than what we could get with 
     * PID controlled rotation lol, see PIDVisionDriveCommand for attempt at that.
     * Extends DefaultDriveCommand and uses it's drive functions, but overrides the turn value with
     * using the limelight to center the crosshair on the hub.
     * @param drivetrain
     * @param translationXSupplier
     * @param translationYSupplier
     * @param vision
     * @param shooter
     * @param tempFieldCentricButtonPressed
     */
    public VisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
        BooleanSupplier tempFieldCentricButtonPressed
    ) {
        super(
            drivetrain,
            translationXSupplier,
            translationYSupplier,
            () -> { return 0.0; }, // This value will not be used because getTurn will be overriden.
            tempFieldCentricButtonPressed
        );

        this.vision = vision;
        // this.targeting = TargetingSubsystem.getInstance();
    }

    public VisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision,
        ShooterSubsystem shooter
    ) {
        this(
            drivetrain, 
            translationXSupplier, 
            translationYSupplier, 
            vision, 
            shooter,
            () -> false
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLEDMode(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        Rotation2d horizontalAngle = vision.getXRotation();//.minus(targeting.getDrivingHorizontalFiringOffsetAngleDegrees());      // Horizontal offset from the Limelight's crosshair to target + driving while shooting offset.
        double horizontalAngleDegrees = horizontalAngle.getDegrees();

        if(vision.getHasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset. Uses somewhat arbitrary values but is what we used in every regional and champs, and it worked so...
            if(Math.abs(horizontalAngleDegrees) > 4) {
//                visionRotation = -Math.copySign(Math.PI/2, horizontalAngle);
                visionRotation = -Math.copySign(Math.toRadians(horizontalAngleDegrees * 9) , horizontalAngleDegrees); // Dynamically changes rotation speed to be faster at a larger tx,
                // targeting.setIsLinedUpToShoot(false);
            } else if(Math.abs(horizontalAngleDegrees) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, horizontalAngleDegrees);
                // targeting.setIsLinedUpToShoot(false);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                // targeting.setIsLinedUpToShoot(true);
            }
        } else {
            // Possible todo: Use the gyro to get the possible general direction of the hub and spin towards that angle - would be highly advanced, but possible.
            visionRotation = 0;
        }

        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGETING);
        
        return visionRotation;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLEDMode(LEDMode.OFF);
        LEDSubsystem.getInstance().clearStatusMode();
    }
}
