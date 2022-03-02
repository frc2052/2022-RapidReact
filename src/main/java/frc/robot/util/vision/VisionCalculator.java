// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

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

    // Singleton pattern for making sure only one instance of this class exists that can be accessed easily from anywhere.
    private VisionCalculator() {
        shooterDistanceConfigs = new ArrayList<ShooterDistanceConfig>();
        setupShooterDistanceConfigs();

        visionDistances = new ArrayList<VisionDistance>();
        setupVisionDistances();
    }
    private static VisionCalculator instance;
    public static VisionCalculator getInstance() {
        if (instance == null) {
            instance = new VisionCalculator();
        }
        return instance;
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and the top and bottom motor percent values.
     */
    private void setupShooterDistanceConfigs() {
        shooterDistanceConfigs.add(new ShooterDistanceConfig(2 * 12, 5000, 10000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(3 * 12, 7400, 7400));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(4 * 12, 7900, 7900));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(5 * 12, 8500, 8500));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(6 * 12, 9000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(7 * 12, 10000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(8 * 12, 10500, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(9 * 12, 11500, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(10 * 12, 13000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(11 * 12, 14000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(12 * 12, 15000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(13 * 12, 15000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(14 * 12, 16000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(15 * 12, 16000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(16 * 12, 17000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(17 * 12, 18000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(18 * 12, 19000, 9000));
        // shooterDistanceConfigs.add(new ShooterDistanceConfig(19 * 12, 15700, 9000));
        // shooterDistanceConfigs.add(new ShooterDistanceConfig(20 * 12, 15700, 9000));
        // shooterDistanceConfigs.add(new ShooterDistanceConfig(21 * 12, 15700, 9000));
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and what {@link VisionSubsystem} values for that distance should be.
     */
    private void setupVisionDistances() {
        // Center Targeted
        visionDistances.add(new VisionDistance(2 * 12,  21.1));
        visionDistances.add(new VisionDistance(3 * 12,  14.32));
        visionDistances.add(new VisionDistance(4 * 12, 8.84));
        visionDistances.add(new VisionDistance(5 * 12, 4.58));
        visionDistances.add(new VisionDistance(6 * 12, 0.82));
        visionDistances.add(new VisionDistance(7 * 12, -2.2));
        visionDistances.add(new VisionDistance(8 * 12, -4.79));
        visionDistances.add(new VisionDistance(9 * 12, -7.07));
        visionDistances.add(new VisionDistance(10 * 12, -8.95));
        visionDistances.add(new VisionDistance(11 * 12, -10.66));
        visionDistances.add(new VisionDistance(12 * 12, -12.68));
        visionDistances.add(new VisionDistance(13 * 12, -14.35));
        visionDistances.add(new VisionDistance(14 * 12, -15.49));
        visionDistances.add(new VisionDistance(15 * 12, -16.56));
        visionDistances.add(new VisionDistance(16 * 12, -17.50));
        visionDistances.add(new VisionDistance(17 * 12, -18.63));
        visionDistances.add(new VisionDistance(18 * 12, -19.79));
        visionDistances.add(new VisionDistance(19 * 12, -20.59));
        visionDistances.add(new VisionDistance(20 * 12, -21.33));
        visionDistances.add(new VisionDistance(21 * 12, -22.44));
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

        //System.err.println("TY Val:" + ty);

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
        if (lowerDistance == null && upperDistance == null) {
            return 0;
        } else if (lowerDistance == null) {
            return upperDistance.getDistanceInches();
        } else if (upperDistance == null) {
            return lowerDistance.getDistanceInches();
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


    public ShooterDistanceConfig getShooterConfig(int distanceInches) {
        if(distanceInches <= 0){
            return new ShooterDistanceConfig(0, 0, 0);
        }
        // Lower bound of the estimated shooter configuration given the distance from the target.
        ShooterDistanceConfig lowerDistanceConfig = null;
        // Upper bound of the estimated shooter configuration given the distance from the target.
        ShooterDistanceConfig upperDistanceConfig = null;

        // Finds the upper and lower distance configuration bounds to more accurately find a shooter configuration.
        for (int i = 0; i < shooterDistanceConfigs.size(); i++) {
            if (shooterDistanceConfigs.get(i).getDistanceInches() < distanceInches) {
                lowerDistanceConfig = shooterDistanceConfigs.get(i);
            } else {
                upperDistanceConfig = shooterDistanceConfigs.get(i);
                break;
            }
        }

        // Returns a default shooter configuration if either of the bounds are null
        if (lowerDistanceConfig == null && upperDistanceConfig == null) {
            return shooterDistanceConfigs.get(Constants.Shooter.DEFAULT_ASSUMED_SHOOTER_CONFIG);
        } else if (lowerDistanceConfig == null) {
            return upperDistanceConfig;
        } else if (upperDistanceConfig == null) {
            return lowerDistanceConfig;
        }

        // deltaInches is the difference between the lower and upper pre-measured inches values.
        double deltaInches = lowerDistanceConfig.getDistanceInches() - upperDistanceConfig.getDistanceInches();
        // offsetInches is the difference between our actual current distance inches and the upper distance inches.
        double offsetInches = distanceInches - upperDistanceConfig.getDistanceInches();

        // The percentage of offsetInches that is deltaInches.
        double pct = offsetInches / deltaInches;

        // Calculate the difference between the two top motor velocities from the lookup table entries.
        double deltaTopMotorVelocityTicksPerSecond = upperDistanceConfig.getTopMotorVelocityTicksPerSecond() - lowerDistanceConfig.getTopMotorVelocityTicksPerSecond();
        // Multiple the difference of top motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetTopMotorVelocityTicksPerSecond = deltaTopMotorVelocityTicksPerSecond * pct;

        // Calculate the difference between the two bottom motor velocities from the lookup table entries.
        double deltaBottomMotorVelocityTicksPerSecond = upperDistanceConfig.getTopMotorVelocityTicksPerSecond() - lowerDistanceConfig.getTopMotorVelocityTicksPerSecond();
        // Multiple the difference of bottom motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetBottomMotorVelocityTicksPerSecond = deltaBottomMotorVelocityTicksPerSecond * pct;

        return new ShooterDistanceConfig(
            distanceInches,
            // Add the final offsets to our lower distance so the bottom shooter cofniguration can safely be assumed.
            lowerDistanceConfig.getBottomMotorVelocityTicksPerSecond() + offsetBottomMotorVelocityTicksPerSecond,
            // Add the final offsets to our lower distance so the top shooter cofniguration can safely be assumed.
            lowerDistanceConfig.getTopMotorVelocityTicksPerSecond() + offsetTopMotorVelocityTicksPerSecond
        );
    }
}
