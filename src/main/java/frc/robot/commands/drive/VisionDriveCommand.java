package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;

public class VisionDriveCommand extends DefaultDriveCommand {
    private VisionSubsystem vision;
    private DrivetrainSubsystem driveTrain;
    private double visionRotation = 0;
    private double horizontalAngle;
    private boolean isLinedUp;

    public VisionDriveCommand(
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
        this.driveTrain = drivetrain;
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setLED(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
        horizontalAngle = vision.getTx() + drivingHorizontalFiringOffsetAngleDegrees();      // Horizontal offset from the Limelight's crosshair to target.
        isLinedUp = false;

        if(vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(horizontalAngle) > 4) {
                visionRotation = -Math.copySign(Math.toRadians(horizontalAngle * 9) , horizontalAngle); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(horizontalAngle) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, horizontalAngle);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
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
        if(driveTrain.getLastWheelVelocity() < 0.2) {    // Just avoids doing all the math if we're not or barely moving anyway
            return 0.0;
        }
        // Ends up using theta = tan^-1(d*(velocity of the robot)/(x velocity of the ball leaving the shooter)/sqrt(height^2+distance^2)) to calculate offset angle.
        int distanceInches = VisionCalculator.getInstance().getDistanceInches(vision.getTy());
        double distanceMeters = Units.inchesToMeters(distanceInches);
        ShooterDistanceConfig shooterDistanceConfig = VisionCalculator.getInstance().getShooterConfig(distanceInches);
        double averageFiringVelocityTP100MS = (shooterDistanceConfig.getTopMotorVelocityTicksPerSecond() + shooterDistanceConfig.getBottomMotorVelocityTicksPerSecond()) / 2;
        double averageFiringVelocityMPS = 0; // TODO Figure out how to convert from ticks per 100 miliseconds to Meters per second
        double lineToHubDistanceMeters = Math.sqrt(Math.pow(Constants.Field.UPPER_HUB_HEIGHT_METERS,2) + Math.pow(distanceMeters, 2));
        double currentWheelVelocityMPS = driveTrain.getLastWheelVelocity();
        double offsetRadians = Math.atan(Math.toRadians(distanceMeters*currentWheelVelocityMPS/averageFiringVelocityMPS/lineToHubDistanceMeters));
        return Math.toDegrees(offsetRadians);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        vision.setLED(LEDMode.OFF);
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT);
    }
}
