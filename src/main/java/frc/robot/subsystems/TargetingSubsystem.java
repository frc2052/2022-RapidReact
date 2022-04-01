package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.VisionCalculator;

// public enum Mode {
//     NONE,
//     NORMAL,
//     OFFSET
// }

public class TargetingSubsystem extends SubsystemBase {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final ShooterSubsystem shooter;

    private TargetingMode targetingMode;
    private Rotation2d currentRotation;
    private Rotation2d rotation;
    private boolean isLinedUpToShoot;

    private TargetingSubsystem(VisionSubsystem vision, DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        isLinedUpToShoot = false;
        rotation = Rotation2d.fromDegrees(0);
    }
    private static TargetingSubsystem instance;
    public static void init(VisionSubsystem vision, DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
        if (instance == null) {
            instance = new TargetingSubsystem(vision, drivetrain, shooter);
        }
    }
    public static TargetingSubsystem getInstance() {
        if (instance == null) {
            System.err.println("ATTEMPTED TO GET TARGETING SUBSYSTEM INSTANCE WHEN NULL");
        }
        return instance;
    }


    // private Mode mode = Mode.NORMAL;
    // public void setMode(Mode mode) {
    //     this.mode = mode;
    // }

    public Rotation2d getDrivingHorizontalFiringOffsetAngleDegrees() {
        if(drivetrain.getIntendedCurrentVelocity() < 0.1) {    // Just avoids doing all the math if we're not or barely moving anyway
            return Rotation2d.fromDegrees(0.0);
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
        double currentWheelVelocityMPS = drivetrain.getIntendedYVelocityMPS(); // Gets the current target velocity for the chasis being sent to the drivetrain's drive method
        double offsetRadians = Math.atan(Math.toRadians(shooterDistanceMeters*currentWheelVelocityMPS/xLaunchVelocity/lineToHubDistanceMeters));  // Plugs everything needed into the equation TODO Figure out if the Math.toRadians needs to be omitted
        rotation = Rotation2d.fromDegrees(Math.toDegrees(offsetRadians));
        return rotation;
        // return 0.0; // doesn't work yet
    }

    public Rotation2d createHubTrackingSupplierWithOffset(double noTargetAngle, double offsetAngle) {
        if(vision.getLedMode() != 3.0) {
            vision.setLEDMode(LEDMode.ON);
        }
        if(vision.getHasValidTarget()) {
            double rotationDegrees = vision.getTx() + offsetAngle;
            rotation = drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(rotationDegrees));
        } else {
            rotation = Rotation2d.fromDegrees(noTargetAngle);
        }
        return rotation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    // public Rotation2d getRotation() {

    // }

    // public void somewhereElse() {
    //     Supplier<Rotation2d> supplier = this::getRotation;
    // }

    // public boolean getIsLinedUpToShoot() {
    //     if (Math.abs(drivetrain.getIntendedCurrentVelocity()) < 0.1) {
    //         return vision.getIsLinedUp();
    //     }
    //     return isLinedUpToShoot;
    // }

    public boolean getIsLinedUpToShoot() {
        if (Math.abs(rotation.getDegrees()) <= vision.getTolerance()) { // currentRotation.minus(rotation).getDegrees()
            isLinedUpToShoot = true;
        } else {
            isLinedUpToShoot = false;
        }
        return isLinedUpToShoot;
    }

    public void setTargetingMode(TargetingMode targetingMode) {
        this.targetingMode = targetingMode;
    }

    // @Override
    // public void periodic() {
    //     currentRotation = drivetrain.getGyroscopeRotation();

    //     if (targetingMode != TargetingMode.NONE) {
    //         if (Math.abs(rotation.getDegrees()) <= vision.getTolerance()) { // currentRotation.minus(rotation).getDegrees()
    //             isLinedUpToShoot = true;
    //         } else {
    //             isLinedUpToShoot = false;
    //         }
    //     } else {
    //         isLinedUpToShoot = false;
    //     }

    //     // switch (targetingMode) {
    //     //     case CALCULATED_DRIVE_AND_SHOOT:
    //     //         if (currentRotation.minus(rotation).getDegrees() <= vision.getTolerance()) {
    //     //             isLinedUpToShoot = true;
    //     //         } else {
    //     //             isLinedUpToShoot = false;
    //     //         }
    //     //         break;
    //     //     case TRACK_HUB_WITH_OFFSET:
    //     //         if (currentRotation.minus(rotation).getDegrees() <= vision.getTolerance()) {
    //     //             isLinedUpToShoot = true;
    //     //         } else {
    //     //             isLinedUpToShoot = false;
    //     //         }
    //     //         break;
    //     //     case NONE:
    //     //         break;
    //     //     default:
    //     //         System.err.println("TARGETING MODE EXECUTE SWITCH FELL THROUGH");
    //     //         break;
    //     // }
    // }

    public void setIsLinedUpToShoot(boolean isLinedUpToShoot) {
        this.isLinedUpToShoot = isLinedUpToShoot;
    }

    public enum TargetingMode {
        CALCULATED_DRIVE_AND_SHOOT,
        TRACK_HUB_WITH_OFFSET,
        NONE
    }
}
