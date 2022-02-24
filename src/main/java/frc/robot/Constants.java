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
//        public static final int ARM_MOTOR = 14;
//        public static final int PITCH_MOTOR = 91;
        public static final int CLIMBER_MOTOR = 14;
        public static final int TOP_SHOOTER_MOTOR = 15;
        public static final int BOTTOM_SHOOTER_MOTOR = 16;
        public static final int HOPPER_MOTOR = 17;
        public static final int INDEXER_MOTOR = 18; //GREEN
        public static final int INDEXER_MOTOR_FEEDER = 19; 
        public static final int INTAKE_MOTOR = 20;
    }
    public static final class Solenoids {
        // Used for claw climber.
        // public static final int CLOSE_A_SOLENOID = 0;
        // public static final int CLOSE_B_SOLENOID = 2;
        // public static final int OPEN_A_SOLENOID = 1;
        // public static final int OPEN_B_SOLENOID = 3;

        public static final int COMPRESSOR_MODULE_ID = 13;

        public static final int CLIMBER_FORWARD_SOLENOID = 2;
        public static final int CLIMBER_BACKWARD_SOLENOID = 3;
        
        public static final int CLIMBER_LOCK_SOLENOID = 0;
        public static final int CLIMBER_UNLOCK_SOLENOID = 1;

        public static final int INTAKE_OUT_SOLENOID = 5;
        public static final int INTAKE_IN_SOLENOID = 4;
    }
    public static final class LimitSwitch {
        public static final int CLAW_A_LIMIT_SWITCH = 7;
        public static final int CLAW_B_LIMIT_SWITCH = 2;
        public static final int INDEXER_PRELOAD = 9;
        public static final int INDEXER_FEEDER = 8;

    }

    public static final class ShooterSub {
        public static final double INDEXER_SPEED = .75;
        public static final double TOP_WHEEL_SPEED = 5;
        public static final double BOTTOM_WHEEL_SPEED = 5;
        public static final double FEEDER_SPEED = 1;

        public static final double SHOOTER_TOLERANCE = 0.05;

        public static final double SHOOTER_FIRING_ANGLE_DEGREES = 72.0;
        public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);

        public static final double SHOOTER_MOUNT_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final int DEFAULT_ASSUMED_SHOOTER_CONFIG = 0;

        public static final double FAR_RANGE_LIMIT_FROM_HUB_METERS = 10; // TEMP VALUE
        public static final double CLOSE_RANGE_LIMIT_FROM_HUB_METERS = 0.5; // TEMP VALUE
    }

    public static final class Field {
        public static final double UPPER_HUB_HEIGHT_METERS = 2.64;
    }

    public static final class Limelight {
        public static final double MOUNT_HEIGHT_METERS = Units.inchesToMeters(21); // ESTIMATE Based on Cad Model, update when robot is completed.
        public static final double MOUNT_ANGLE_DEGREES = 45;

        public static final double LINED_UP_THRESHOLD = 2;

        public static final double DISTANCE_CALCULATION_LINEAR_OFFSET = 0;

        public static final int DEFAULT_ASSUMED_DISTANCE = 7;

        public static final int RELAY_PORT = 0;
    }

    public static final class PixyCamConstants {
        public static final double FOV = 60;
        public static final double IMAGE_WIDTH_PIXELS = 316;
    }

    public static final class DriveTrain {
        // The left-to-right distance between the drivetrain wheels
        // Should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
        // The front-to-back distance between the drivetrain wheels.
        // Should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(218.1);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 8;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(176.5);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(45.9);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(61.1);

        public static final double DRIVING_AIM_ANGLE_OFFSET_DEGREES = 0.0;
    }

    public static final class Intake {
        //sets the speed of the motors
        public static final double kIntakeSpeed = .75;
        public static final double kHopperSpeed = .75;
    }

    public static final class Climber {
        public static final double CLIMBER_EXTENSION_SPEED_PCT  = 0.4; // .5
        public static final double CLIMBER_RETRACT_SPEED_PCT  = -0.4; // .75
        public static final double WINCH_CIRCUMFERENCE_INCHES = .8 * Math.PI;
        // our gearbox ratio is 12
        public static final double TICKS_PER_WINCH_ROTATION = 2048 / 12;

        // TODO: These Value Needs To Change
        public static final double MIN_CLIMBER_HEIGHT_TICKS = 1000;
        public static final double MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 250000;
        public static final double MAX_CLIMBER_HEIGHT_TICKS_TILTED = 335000;
    }

    public static final class LEDs {
        public static final int CANIFIER_PORT = 21;
    }
}
