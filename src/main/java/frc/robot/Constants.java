// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Motors {
        public static final int kTurretMotorID = 12;
    }

    //Copied from 2020 Scorpion for testing aiming using Limelight
    public static final class Turret{
        public static final double kTurnLeftSpeed = -0.1;
        public static final double kTurnRightSpeed = 0.1;

        public static final int kTurretMinEncoderPos = -4000;
        public static final int kTurretMaxEncoderPos = 16000;

        public static final double kTicksPerDegree = 4096 / 90;
        public static final double kMaxAngle = 90;
        public static final double kMinAngle = -90;

        public static final double kMinTurretSpeed = .07;
        public static final double kMaxTurretSpeed = .75;
    }
  
}
