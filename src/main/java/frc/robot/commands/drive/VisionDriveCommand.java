package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.VisionCalculator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;

public class VisionDriveCommand extends DefaultDriveCommand {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private double visionRotation = 0;
    private double horizontalAngle;
    private boolean isLinedUp;

    public VisionDriveCommand(
        DrivetrainSubsystem drivetrain,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
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
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLED(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        horizontalAngle = vision.getTx() - drivingHorizontalFiringOffsetAngleDegrees();      // Horizontal offset from the Limelight's crosshair to target.
        vision.setExternalIsLinedUp(false);
        isLinedUp = false;

        if(vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(horizontalAngle) > 4) {
//                visionRotation = -Math.copySign(Math.PI/2, horizontalAngle);
                visionRotation = -Math.copySign(Math.toRadians(horizontalAngle * 9) , horizontalAngle); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(horizontalAngle) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, horizontalAngle);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                vision.setExternalIsLinedUp(true);
                isLinedUp = true;
            }
        } else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            visionRotation = 0;
        }

        if (isLinedUp) {
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_FOUND);
        } else {
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGETING);
        }
        return visionRotation;
    }

    public boolean getIsLinedUp() {
        return isLinedUp;
    }

    private double drivingHorizontalFiringOffsetAngleDegrees() {
        if(drivetrain.getIntendedCurrentVelocity() < 0.2) {    // Just avoids doing all the math if we're not or barely moving anyway
            return 0.0;
        }
        // Ends up using theta = tan^-1(d*(velocity of the robot)/(x velocity of the ball leaving the shooter)/sqrt(height^2+distance^2)) to calculate offset angle.
        int distanceInches = VisionCalculator.getInstance().getDistanceInches(vision.getTy());  // Gets the calculated distance from the VisionCalculator
        double distanceMeters = Units.inchesToMeters(distanceInches);   // Converts to meters
        double shooterDistanceMeters = distanceMeters + Constants.Field.EDGE_TO_CENTER_OF_HUB_OFFSET_METERS - Constants.Limelight.OFFSET_FROM_CENTER_SHOOTER.getX();
        // ShooterDistanceConfig shooterDistanceConfig = VisionCalculator.getInstance().getShooterConfig(distanceInches);  // Gets the shooter config for the distance, so we can get the wheel velocities
        // double averageFiringVelocityTP100MS = (shooterDistanceConfig.getTopMotorVelocityTicksPerSecond() + shooterDistanceConfig.getBottomMotorVelocityTicksPerSecond()) / 2;   // Finds average velocity between the 2 shooter wheel velocities
        // double averageFiringVelocityMPS = averageFiringVelocityTP100MS * 10 / 2048 * 2 * Math.PI * Constants.Shooter.FLYWHEEL_RADIUS_METERS; // Ticks per 100 ms to rotations per second by multiplying by 10 to get to seconds, and deviding by 2048, the amount of ticks per revolution of the falcon shooter motors. Can use this raw becasue the gear ratio from the motore to the shooter should be 1:1.

        double launchVelocity = Math.sqrt((shooterDistanceMeters * 9.8) / Math.sin(Math.toRadians(2 * shooter.getShootAngleDegrees()))); // Gets the velocity we should be launching the ball at
        double xLaunchVelocity = launchVelocity * Math.cos(Math.toRadians(shooter.getShootAngleDegrees())); //TODO Change to get current shooter firing angle or somthing
        // double yLaunchVelocity = launchVelocity * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_FIRING_ANGLE_DEGREES));


        double lineToHubDistanceMeters = Math.hypot(Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.Limelight.MOUNT_HEIGHT_METERS , shooterDistanceMeters); // Gets the length for t
        double currentWheelVelocityMPS = drivetrain.getIntendedCurrentVelocity(); // Gets the current target velocity for the chasis being sent to the drivetrain's drive method
        double offsetRadians = Math.atan(Math.toRadians(shooterDistanceMeters*currentWheelVelocityMPS/xLaunchVelocity/lineToHubDistanceMeters));  // Plugs everything needed into the equation TODO Figure out if the Math.toRadians needs to be omitted
        return Math.toDegrees(offsetRadians);   // Returns the needed offset in degrees
        // return 0.0; // doesn't work yet
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLED(LEDMode.OFF);
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT);
    }
}
