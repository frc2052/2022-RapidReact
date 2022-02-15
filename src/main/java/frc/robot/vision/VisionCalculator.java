// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;

/** 
 * Class for converting {@link VisionSubsystem} ty values to distance (inches) from target
 * and converting distances (inches) to a {@link ShooterDistanceConfig}.
 */
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
        shooterDistanceConfigs.add(new ShooterDistanceConfig(0 * 12, 0.0, 0.0));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(1 * 12, 0.0, 0.0));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(2 * 12, 0.0, 0.0));
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and what {@link VisionSubsystem} values for that distance should be.
     */
    private void setupVisionDistances() {
        visionDistances.add(new VisionDistance(0 * 12, 0.0));
    }

    /**
     * Calculates the distance to the target given a set of {@link VisionSubsystem} values
     * 
     * @param ty the vertical offset from crosshair to target
     * @return the distance to the target
     */
    public int getDistanceInches(double ty) {
        // Lower bound of the estimated distance from the target.
        VisionDistance lowerDistance = null;
        // Upper bound of the estimated distance from the target.
        VisionDistance upperDistance = null;

        // Finds the upper and lower distance bounds to more accurately find a distance.
        for (int i = 0; i < visionDistances.size(); i++) {
            if (visionDistances.get(i).getTY() > ty) {
                lowerDistance = visionDistances.get(i);
            } else {
                upperDistance = visionDistances.get(i);
                break;
            }
        }

        // Returns a default distance if either of the bounds are null.
        if (lowerDistance == null || upperDistance == null) {
            return visionDistances.get(Constants.Limelight.DEFAULT_ASSUMED_DISTANCE).getDistanceInches();
        }

        // deltaY is the difference between the lower and upper pre-measured ty values.
        double deltaY = lowerDistance.getTY() - upperDistance.getTY();
        // offsetY is the difference between our actual current ty and the upper ty.
        double offsetY = ty - upperDistance.getTY();

        // The percentage of offsetY that is deltaY.
        double pct = offsetY / deltaY;
        // Calculate the difference between the two distances from the lookup table entries.
        double deltaInches = upperDistance.getDistanceInches() - lowerDistance.getDistanceInches();
        // Multiple the difference of distances by the percentage of offsetY and deltaY. 
        double offsetInches = deltaInches * pct;

        // Add the final offset to our lower distance so the inches from the target can safely be assumed.
        return (int)(lowerDistance.getDistanceInches() + offsetInches);
    }


    public ShooterDistanceConfig getShooterConfig(int distanceInches){
        // Lower bound of the estimated shooter configuration given the distance from the target.
        ShooterDistanceConfig lowerDistanceConfig = null;
        // Upper bound of the estimated shooter configuration given the distance from the target.
        ShooterDistanceConfig upperDistanceConfig = null;

        // Finds the upper and lower distance configuration bounds to more accurately find a shooter configuration.
        for (int i = 0; i < shooterDistanceConfigs.size(); i++) {
            if (shooterDistanceConfigs.get(i).getDistanceInches() > distanceInches) {
                lowerDistanceConfig = shooterDistanceConfigs.get(i);
            } else {
                upperDistanceConfig = shooterDistanceConfigs.get(i);
                break;
            }
        }

        // Returns a default shooter configuration if either of the bounds are null
        if (lowerDistanceConfig == null || upperDistanceConfig == null) {
            return shooterDistanceConfigs.get(Constants.ShooterSub.DEFAULT_ASSUMED_SHOOTER_CONFIG);
        }

        // deltaInches is the difference between the lower and upper pre-measured inches values.
        double deltaInches = lowerDistanceConfig.getDistanceInches() - upperDistanceConfig.getDistanceInches();
        // offsetInches is the difference between our actual current distance inches and the upper distance inches.
        double offsetInches = distanceInches - upperDistanceConfig.getDistanceInches();

        // The percentage of offsetInches that is deltaInches.
        double pct = offsetInches / deltaInches;

        // Calculate the difference between the two top motor percents from the lookup table entries.
        double deltaTopMotorPercent = upperDistanceConfig.getTopMotorPct() - lowerDistanceConfig.getTopMotorPct();
        // Multiple the difference of top motor percents by the percentage of offsetInches and deltaInches. 
        double offsetTopMotorPercent = deltaTopMotorPercent * pct;

        // Calculate the difference between the two bottom motor percents from the lookup table entries.
        double deltaBottomMotorPercent = upperDistanceConfig.getTopMotorPct() - lowerDistanceConfig.getTopMotorPct();
        // Multiple the difference of bottom motor percents by the percentage of offsetInches and deltaInches. 
        double offsetBottomMotorPercent = deltaBottomMotorPercent * pct;

        return new ShooterDistanceConfig(
            distanceInches,
            // Add the final offsets to our lower distance so the bottom shooter cofniguration can safely be assumed.
            lowerDistanceConfig.getBottomMotorPct() + offsetBottomMotorPercent,
            // Add the final offsets to our lower distance so the top shooter cofniguration can safely be assumed.
            lowerDistanceConfig.getTopMotorPct() + offsetTopMotorPercent
        );
    }
}
