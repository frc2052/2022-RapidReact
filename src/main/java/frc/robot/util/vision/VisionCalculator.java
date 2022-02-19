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
        shooterDistanceConfigs.add(new ShooterDistanceConfig(2 * 12, 7800, 7800));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(3 * 12, 7800, 7800));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(4 * 12, 8500, 8500));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(5 * 12, 9000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(6 * 12, 9700, 8800));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(7 * 12, 10000, 8800));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(8 * 12, 11000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(9 * 12, 12000, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(10 * 12, 12700, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(11 * 12, 13300, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(12 * 12, 14400, 9000));
        shooterDistanceConfigs.add(new ShooterDistanceConfig(13 * 12, 15700, 9100));
    }

    /**
     * Sets up value map between distance from the target (inches)
     * and what {@link VisionSubsystem} values for that distance should be.
     */
    private void setupVisionDistances() {
        // Center Targeted
        visionDistances.add(new VisionDistance(2 * 12,  6.637419700622559));
        visionDistances.add(new VisionDistance(3 * 12,  1.673193931579590));
        visionDistances.add(new VisionDistance(4 * 12, -3.041108131408691));
        visionDistances.add(new VisionDistance(5 * 12, -6.792672157287598));
        visionDistances.add(new VisionDistance(6 * 12, -10.15470218658447));
        visionDistances.add(new VisionDistance(7 * 12, -13.33554172515869));
        visionDistances.add(new VisionDistance(8 * 12, -15.87055301666260));
        visionDistances.add(new VisionDistance(9 * 12, -18.17145729064941));
        visionDistances.add(new VisionDistance(10 * 12, -20.06331825256348));
        visionDistances.add(new VisionDistance(11 * 12, -21.58301734924316));
        visionDistances.add(new VisionDistance(12 * 12, -23.56243324279785));
        visionDistances.add(new VisionDistance(13 * 12, -24.5762882232666));
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

        System.err.println("TY Val:" + ty);

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
            System.err.println("IT BROKEN!!!!!!!!!!!!!!!!!!!!!!");
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


    public ShooterDistanceConfig getShooterConfig(int distanceInches) {
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
        if (lowerDistanceConfig == null || upperDistanceConfig == null) {
            //System.err.println("IT ALSO SHOOTER BROKEN!!!!!!!!!!!!!!!!!!!!!!");
            return shooterDistanceConfigs.get(Constants.ShooterSub.DEFAULT_ASSUMED_SHOOTER_CONFIG);
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
