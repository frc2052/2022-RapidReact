// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/** 
 * Wrapper class for {@link VisionSubsystem} values, 
 * used by {@link VisionCalculator} to store values for motor percents.
 */
public class VisionDistance {
    private int distanceMappingInches;
    private double ty;
    private double ta;
    private double ts;
    private double thor;

    public VisionDistance(int distanceMappingInches, double ty, double ta, double ts, double thor) {
        this.distanceMappingInches = distanceMappingInches;
        this.ty = ty;
        this.ta = ta;
        this.ts = ts;
        this.thor = thor;
    }

    public int getDistanceMappingInches() {
        return distanceMappingInches;
    }

    public double getTY() {
        return ty;
    }

    public double getTA() {
        return ta;
    }

    public double getTS() {
        return ts;
    }

    public double getTHOR() {
        return thor;
    }
}