// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * All distance constants should be stored in meters.
 */
public final class Constants {
    public static final class Field {
        public static final double UPPER_HUB_HEIGHT_METERS = 2.64;
        public static final double EDGE_TO_CENTER_OF_HUB_OFFSET_METERS = 0.68;
    }

    public static final class MotorIDs {
        public static final int CLIMBER_MOTOR = 14;
        public static final int TOP_SHOOTER_MOTOR = 15;
        public static final int BOTTOM_SHOOTER_MOTOR = 16;
        public static final int HOPPER_MOTOR = 17; // Horizontal blue compliant wheels
        public static final int PRELOAD_INDEXER_MOTOR = 18; // Green compliant wheel
        public static final int FEEDER_INDEXER_MOTOR = 19; // Small blue wheel directly under shooter
        public static final int INTAKE_MOTOR = 20;
    }

    public static final class SwerveModule {
        /*
         * The maximum voltage that will be delivered to the drive motors.
         * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
        public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
        public static final int TICKS_PER_ROTATION = 2048;

        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * By default this value is setup for a Mk3 standard module using Falcon500s to drive.
         * An example of this constant for a Mk4 L2 module with NEOs to drive is:
         * 5880.0 (RPM) / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            DRIVE_REDUCTION * WHEEL_DIAMETER_METERS * Math.PI;
        /*
         * The theoretical maximum angular velocity of the robot in radians per second.
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(
            Constants.SwerveDriveTrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
            Constants.SwerveDriveTrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );
    }

    public static final class SwerveDriveTrain {
        // The left-to-right distance between the drivetrain wheels
        // Should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
        // The front-to-back distance between the drivetrain wheels.
        // Should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(218.1);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 8;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(176.5);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(39); //45.9 - old value, changed to straighten back left

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(61.1);
    }

    public static final class Solenoids {
        public static final int COMPRESSOR_MODULE_ID = 13;

        public static final int CLIMBER_FORWARD_SOLENOID = 2;
        public static final int CLIMBER_BACKWARD_SOLENOID = 3;
        
        public static final int CLIMBER_LOCK_SOLENOID = 0;
        public static final int CLIMBER_UNLOCK_SOLENOID = 1;

        public static final int INTAKE_OUT_SOLENOID = 5;
        public static final int INTAKE_IN_SOLENOID = 4;

        public static final int SHOOTER_ANGLE_OUT_SOLENOID = 7;
        public static final int SHOOTER_ANGLE_IN_SOLENOID = 6;
    }

    public static final class LimitSwitch {
        public static final int CLAW_A_LIMIT_SWITCH = 7;
        public static final int CLAW_B_LIMIT_SWITCH = 2;

        public static final int INDEXER_PRELOAD = 9;
        public static final int INDEXER_FEEDER = 8;
    }

    public static final class Shooter {
        public static final double PRELOAD_WHEEL_SPEED = 1;
        public static final double TOP_WHEEL_SPEED = 5;
        public static final double BOTTOM_WHEEL_SPEED = 5;
        public static final double INDEXER_WHEEL_SPEED = 0.75;

        public static final double SHOOTER_TOLERANCE = 0.01;
        public static final double SHOOTER_TOP_PULLDOWN_PCT = 0.97;
        public static final double SHOOTER_BOTTOM_PULLDOWN_PCT = 0.97;

        // public static final double ANGLE_CHANGE_THRESHOLD_TY = -5.0;
        // public static final double ANGLE_CHANGE_TOLERANCE_DEGREES = 1; //

        public static final double ANGLE_CHANGE_THRESHOLD_DISTANCE_INCHES = 66;
        public static final double ANGLE_CHANGE_TOLERANCE_DISTANCE_INCHES = 6;

        public static final double FIRING_ANGLE_1_DEGREES = 72.0;
        public static final double FIRING_ANGLE_2_DEGREES = 60.0;
        public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);

        public static final double SHOOTER_MOUNT_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final int DEFAULT_ASSUMED_SHOOTER_CONFIG = 0;
    }

    public static final class Limelight {
        public static final double MOUNT_HEIGHT_METERS = Units.inchesToMeters(28);
        public static final double MOUNT_ANGLE_DEGREES = 65;

        public static final double LINED_UP_THRESHOLD = 2;

        public static final double DISTANCE_CALCULATION_LINEAR_OFFSET = 0;
        public static final int DEFAULT_ASSUMED_DISTANCE = 7;

        public static final int RELAY_PORT = 0;

        public static final double PIPELINE_SWITCH_TY_DEGREES = -20;
        public static final double PIPELINE_SWITCH_THRESHOLD = 3;

        public static final double FAR_RANGE_FROM_HUB_ANGLE_DEGREES = -22.4;
        public static final double CLOSE_RANGE_FROM_HUB_ANGLE_DEGREES = 21.1;

        public static final double DEFAULT_PIPELINE = 0.0;

        public static final Translation2d OFFSET_FROM_CENTER_SHOOTER = new Translation2d(7.5, 8);
    }

    public static final class PixyCamConstants {
        public static final double FOV = 60;
        public static final double IMAGE_WIDTH_PIXELS = 316;
    }

    public static final class Intake {  
        // Sets the speed of the motors
        public static final double kIntakeSpeed = .90;
        public static final double kHopperSpeed = .75;
    }

    public static final class Climber {
        public static final double CLIMBER_EXTENSION_SPEED_PCT  = 1; // .5
        public static final double CLIMBER_RETRACT_SPEED_PCT  = -.8; // .75
        public static final double WINCH_CIRCUMFERENCE_INCHES = .8 * Math.PI;
        // Gearbox ratio is 20
        public static final double TICKS_PER_WINCH_ROTATION = 2048 / 12;

        public static final double MIN_CLIMBER_HEIGHT_TICKS = -10000; // Allow to climb below zero because under weight, the strap will wind tigher.
        public static final double MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 220000;
        public static final double MAX_CLIMBER_HEIGHT_TICKS_TILTED = 300000;
    }

    public static final class LEDs {
        // public static final int CANIFIER_PORT = 21;

        // For binary arduino code output
        public static final int CHANNEL_1_PIN = 0; // 2^0
        public static final int CHANNEL_2_PIN = 1; // 2^1
        public static final int CHANNEL_3_PIN = 2; // 2^2
        public static final int CHANNEL_4_PIN = 3; // 2^3
        public static final int CHANNEL_5_PIN = 4; // 2^4

    }

    public static final class Misc {
        public static final double USB_CAM_MOUNT_HEIGHT_METERS = Units.inchesToMeters(29.5);
    }
}
