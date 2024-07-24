// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swervemodule;

/** Add your docs here. */
public class SwerveConstants {
    public static final double MAX_VOLTAGE_VOLTS = 12.0;

    public static final class SwerveModule {
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
        public static final boolean DRIVE_INVERTED = true;
        public static final double ANGLE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
        public static final boolean ANGLE_INVERTED = false;

        public static final int FALCON500_ROUNDS_PER_MINUTE = 6380;
        public static final int TICKS_PER_ROTATION = 2048;

        public static final double ANGLE_MOTOR_P = 0.2;
        public static final double ANGLE_MOTOR_I = 0.0;
        public static final double ANGLE_MOTOR_D = 0.1;
    }
}
