// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

/** 
 * Wrapper class for a pair of shooter motor percents, 
 * used by {@link VisionCalculator} to store values for motor percents. Top motor velocity first then bottom
 */
public class ShooterDistanceConfig {
    private final int distanceInches;
    private final double topMotorVelocityTicksPerSecond;
    private final double bottomMotorVelocityTicksPerSecond; 

    public ShooterDistanceConfig(int distanceInches, double topMotorVelocityTicksPerSecond, double bottomMotorVelocityTicksPerSecond) {
        this.distanceInches = distanceInches;
        this.topMotorVelocityTicksPerSecond = topMotorVelocityTicksPerSecond;
        this.bottomMotorVelocityTicksPerSecond = bottomMotorVelocityTicksPerSecond;
    }

    public double getTopMotorVelocityTicksPerSecond() {
        return topMotorVelocityTicksPerSecond;
    }

    public int getDistanceInches() {
        return this.distanceInches;
    }

    public double getBottomMotorVelocityTicksPerSecond() {
        return bottomMotorVelocityTicksPerSecond;
    }
}
