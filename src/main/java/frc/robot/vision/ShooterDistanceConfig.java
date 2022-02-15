// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/** 
 * Wrapper class for a pair of shooter motor percents, 
 * used by {@link VisionCalculator} to store values for motor percents.
 */
public class ShooterDistanceConfig {
    private final int distanceInches;
    private final double bottomMotorPct; 
    private final double topMotorPct;

    public ShooterDistanceConfig(int distanceInches, double bottomMotorPct, double topMotorPct) {
        this.distanceInches = distanceInches;
        this.bottomMotorPct = bottomMotorPct;
        this.topMotorPct = topMotorPct;
    }

    public int getDistanceInches() {
        return this.distanceInches;
    }

    public double getBottomMotorPct() {
        return bottomMotorPct;
    }

    public double getTopMotorPct() {
        return topMotorPct;
    }
}