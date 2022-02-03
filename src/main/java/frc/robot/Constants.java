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
    } 

    public static final class ShooterSub {
        public static final double INDEXER_SPEED = .75;
        public static final double TOPWHEELSPEED = 5;
        public static final double BOTTOMWHEELSPEED = 5;
    }
    
    public static final class Field {
        public static final double kUpperHubHeight = 2.64;
    }

    public static final class Limelight {
        public static final double kMountHeight = Units.inchesToMeters(19);
        public static final double kMountAngle = 45;

        public static final double kDistanceCalcOffset = 0;
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
    }
    
    public static final class Intake{
        public static final double kIntakeSpeed = .75;
        public static final double kHopperSpeed = .75;
        //sets the speed of the motors
    
    }
}
