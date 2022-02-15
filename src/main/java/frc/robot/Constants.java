// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
// Constants shall be stored in meters
public final class Constants {
    public static final class MotorIDs {
        public static final int PITCH_MOTOR = 91;
        public static final int TOP_SHOOTER_MOTOR = 92;
        public static final int BOTTOM_SHOOTER_MOTOR = 93;
        public static final int INDEXER_MOTOR = 94;
        public static final int INDEXER_MOTOR_FEEDER = 95;
        public static final int ARM_MOTOR = 87;
        public static final int CLIMBER_MOTOR = 88;
        public static final int INTAKE_MOTOR = 76;
        public static final int HOPPER_MOTOR = 77;
    } 
    public static final class Solenoids {
        // Used for claw climber.
        // public static final int CLOSE_A_SOLENOID = 0;
        // public static final int CLOSE_B_SOLENOID = 2;
        // public static final int OPEN_A_SOLENOID = 1;
        // public static final int OPEN_B_SOLENOID = 3;

        public static final int CLIMBER_FORWARD_SOLENOID = 0;
        public static final int CLIMBER_BACKWARD_SOLENOID = 1;
        public static final int CLIMBER_LOCK_SOLENOID = 2;
        public static final int CLIMBER_UNLOCK_SOLENOID = 3;

        public static final int INTAKE_OUT_SOLENOID = 5;
        public static final int INTAKE_IN_SOLENOID = 0;
    } 
    public static final class LimitSwitch {
        public static final int CLAW_A_LIMIT_SWITCH = 7;
        public static final int CLAW_B_LIMIT_SWITCH = 2;
        public static final int INDEXER_PRELOAD = 0;
        public static final int INDEXER_FEEDER = 1;
    
    } 

    public static final class ShooterSub {
        public static final double INDEXER_SPEED = .75;
        public static final double TOP_WHEEL_SPEED = 5;
        public static final double BOTTOM_WHEEL_SPEED = 5;
        public static final double FEEDER_SPEED = .75;

        public static final double SHOOTER_FIRING_ANGLE_DEGREES = 72.0;
        public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);

        public static final double SHOOTER_MOUNT_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final int DEFAULT_ASSUMED_SHOOTER_CONFIG = 0;
    }
    
    public static final class Field {
        public static final double UPPER_HUB_HEIGHT_METERS = 2.64;
    }

    public static final class Limelight {
        public static final double MOUNT_HEIGHT_METERS = Units.inchesToMeters(21); // ESTIMATE Based on Cad Model, update when robot is completed.
        public static final double MOUNT_ANGLE_DEGREES = 45;

        public static final double DISTANCE_CALCULATION_LINEAR_OFFSET = 0;

        public static final int DEFAULT_ASSUMED_DISTANCE = 0;
    }

    public static final class PixyCamConstants {
        public static final double FOV = 60;
        public static final double IMAGE_WIDTH_PIXELS = 316;
    }

    public static final class DriveTrain{
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(16);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(25.5);


        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(36.241150 + 180);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(355.187988 + 180);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(42.942810 + 180);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(241.136169 + 180);

        public static final double DRIVING_AIM_ANGLE_OFFSET_DEGREES = 0.0;
    }
    
    public static final class Intake{
        public static final double kIntakeSpeed = .75;
        public static final double kHopperSpeed = .75;
        //sets the speed of the motors
    
    }

    public static final class Climber {

        public static final double CLIMBER_EXTENSION_SPEED  = .75;
        public static final double WINCH_CIRCUMFERENCE_INCHES = .8 * Math.PI;
        // our gearbox ratio is 12
        public static final double TICKS_PER_WINCH_ROTATION = 2048 / 12;

        // TODO: These Value Needs To Change
        public static final double MIN_CLIMBER_HEIGHT_TICKS = 0;
        public static final double MAX_CLIMBER_HEIGHT_TICKS = 100;
    }

    public static final class LEDs {
        public static final int CANIFIER_PORT = 40;
    }
}
