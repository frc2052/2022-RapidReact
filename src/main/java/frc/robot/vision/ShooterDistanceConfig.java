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
    private final double topMotorPct;
    private final double bottomMotorPct; 

    public ShooterDistanceConfig(int distanceInches, double topMotorPct, double bottomMotorPct) {
        this.distanceInches = distanceInches;
        this.topMotorPct = topMotorPct;
        this.bottomMotorPct = bottomMotorPct;
    }

    public int getDistanceInches() {
        return this.distanceInches;
    }

    public double getTopMotorPct() {
        return topMotorPct;
    }

    public double getBottomMotorPct() {
        return bottomMotorPct;
    }
}
