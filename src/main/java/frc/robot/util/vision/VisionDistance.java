// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

/** 
 * Wrapper class for {@link VisionSubsystem} values, 
 * used by {@link VisionCalculator} to store values for motor percents.
 */
public class VisionDistance {
    private int distanceInches;
    private double ty;

    public VisionDistance(int distanceMappingInches, double ty) {
        this.distanceInches = distanceMappingInches;
        this.ty = ty;
    }

    public int getDistanceInches() {
        return distanceInches;
    }

    public double getTY() {
        return ty;
    }
}