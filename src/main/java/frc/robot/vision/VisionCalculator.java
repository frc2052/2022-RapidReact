// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class VisionCalculator {
    // Map of distance (inches) to shooter distance configs
    private final List<ShooterDistanceConfig> shooterDistanceConfigs;
    // Map of limelight values to distances (inches)
    private final List<VisionDistance> visionDistances;

    public VisionCalculator() {
        shooterDistanceConfigs = new ArrayList<ShooterDistanceConfig>();
        setupShooterDistanceConfigs();

        visionDistances = new ArrayList<VisionDistance>();
        setupVisionDistances();
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and the top and bottom motor percent values.
     */
    private void setupShooterDistanceConfigs() {
        shooterDistanceConfigs.add(new ShooterDistanceConfig(0, 0.0, 0.0));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(1, 0.0, 0.0));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(2, 0.0, 0.0));
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and what {@link VisionSubsystem} values for that distance should be.
     */
    private void setupVisionDistances() {
        visionDistances.add(new VisionDistance(0, 0.0, 0.0, 0.0, 0.0));
    }

    /**
     * Calculates the distance to the target given a set of {@link VisionSubsystem} values
     * TODO: This code is borrowed from the 2020 Scorpion code and needs to be tested
     * 
     * @param ty
     * @param ta
     * @param ts
     * @param thor
     * @return the distance to the target
     */
    public int getDistanceInches(double ty, double ta, double ts, double thor) {
        VisionDistance lowDistance = null;
        VisionDistance highDistance = null;

        for (int i = 0; i < visionDistances.size(); i++) {
            if (visionDistances.get(i).getTY() > ty) {
                lowDistance = visionDistances.get(i);
            } else {
                highDistance = visionDistances.get(i);
                break;
            }
        }

        if (lowDistance == null || highDistance == null) {
            // TODO: implement default value
            return 8;
        }

        double deltaY = lowDistance.getTY() - highDistance.getTY();
        double offsetY = ty - highDistance.getTY();

        double pct = offsetY / deltaY;
        double deltaInches = highDistance.getDistanceMappingInches() - lowDistance.getDistanceMappingInches();
        double offsetInches = deltaInches * pct;

        int resultingInches = (int)(lowDistance.getDistanceMappingInches() + offsetInches);

        return resultingInches;
    }


    public ShooterDistanceConfig getShooterConfig(int distanceInches){
        
        ShooterDistanceConfig before = null;
        ShooterDistanceConfig after = null;

        for(int i = 0; i < shooterDistanceConfigs.size(); i++){
            if (shooterDistanceConfigs.get(i).getDistanceInches() < distanceInches){
                before = shooterDistanceConfigs.get(i);
            } else {
                after = shooterDistanceConfigs.get(i);
                break;
            }
        }
        
        if(before == null || after == null){
            return shooterDistanceConfigs.get(7); //by default, return the auto line distance
        }

        double deltaInches = before.getDistanceInches() - after.getDistanceInches();
        double offsetInches = distanceInches - after.getDistanceInches();

        // TODO: All this Code is definitly wrong VVVVVVV
        double pct = offsetInches / deltaInches;
        double deltaTopPercent = after.getTopMotorPct() - before.getTopMotorPct();
        double offsetTopPercent = deltaTopPercent * pct;

        double deltaBottomPercent = after.getTopMotorPct() - before.getTopMotorPct();
        double offsetBottomPercent = deltaBottomPercent * pct;

        return new ShooterDistanceConfig((int)offsetInches, offsetTopPercent, offsetBottomPercent);
    }
}
