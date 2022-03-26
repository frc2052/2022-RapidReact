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

    private boolean isLinedUpToShoot;

    private TargetingSubsystem(VisionSubsystem vision, DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        isLinedUpToShoot = false;
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

    public double getDrivingHorizontalFiringOffsetAngleDegrees() {
        if(drivetrain.getIntendedCurrentVelocity() < 0.1) {    // Just avoids doing all the math if we're not or barely moving anyway
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

    public Supplier<Rotation2d> createHubTrackingSupplierWithOffset(double noTargetAngle, double offsetAngle) {
        return () -> {
            if(vision.getLedMode() != 3.0) {
                vision.setLED(LEDMode.ON);
            }
            Rotation2d rotation;
            if(vision.hasValidTarget()) {
                double rotationDegrees = vision.getTx() + offsetAngle;
                rotation = drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(rotationDegrees));
                if (Math.abs(rotation.getDegrees()) <= 3) {
                    isLinedUpToShoot = true;
                } else {
                    isLinedUpToShoot = false;
                }
            } else {
                rotation = Rotation2d.fromDegrees(noTargetAngle);
            }
            return rotation;
        };
    }

    // public Rotation2d getRotation() {

    // }

    // public void somewhereElse() {
    //     Supplier<Rotation2d> supplier = this::getRotation;
    // }

    public boolean getIsLinedUpToShoot() {
        if (Math.abs(drivetrain.getIntendedCurrentVelocity()) < 0.1) {
            return vision.isLinedUp();
        }
        return isLinedUpToShoot;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public void setIsLinedUpToShoot(boolean isLinedUpToShoot) {
        this.isLinedUpToShoot = isLinedUpToShoot;
    }
}
