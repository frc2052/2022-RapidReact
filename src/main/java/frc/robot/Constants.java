// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Field {
      public static final double UPPER_HUB_HEIGHT_METERS = 2.64;
      public static final double EDGE_TO_CENTER_OF_HUB_OFFSET_METERS = 0.68;
  }

  public static final class MotorIDs {
      public static final int TOP_SHOOTER_MOTOR = 15;
      public static final int BOTTOM_SHOOTER_MOTOR = 16;
      public static final int HOPPER_MOTOR = 17; // Horizontal blue compliant wheels
      public static final int PRELOAD_INDEXER_MOTOR = 18; // Green compliant wheel
      public static final int FEEDER_INDEXER_MOTOR = 19; // Small blue wheel directly under shooter
      public static final int INTAKE_MOTOR = 20;
      // public static final int ARM_MOTOR = 14;
      // public static final int PITCH_MOTOR = 91;
  }

  public static final class Solenoids {
      public static final int COMPRESSOR_MODULE_ID = 13;

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

  public static class Drivetrain {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_ANGLE_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_ENCODER = 5;
    public static final double FRONT_LEFT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(217.6);//-12.62054443359375

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_ANGLE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_ENCODER = 8;
    public static final double FRONT_RIGHT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(175.7);//1.79901123046875

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_ANGLE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(39.9);//-360.99700927734375

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_ANGLE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(61.1);//-360.14556884765625
  }

  public static final class Intake {  // Sets the speed of the motors
    public static final double kIntakeSpeed = .90;
    public static final double kHopperSpeed = .75;
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
