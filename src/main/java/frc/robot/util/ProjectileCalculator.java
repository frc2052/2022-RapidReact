package frc.robot.util;

import frc.robot.Constants;

public class ProjectileCalculator {
    
    /**
     * Calculates required velocity to send a projectile to a point with given distance and height relative to launcher, as well as the angle to fire from
     * using the equation V0 = √((-9.8d²)/(2cos²(θ)(h-d*tanθ))).
     * @param distance
     * @return
     */
    public static double calculateReqProjectileVelocity(double distanceMeters) {
        double height = Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.ShooterSub.SHOOTER_MOUNT_HEIGHT_METERS;
        double angleRadians = Math.toRadians(Constants.ShooterSub.SHOOTER_FIRING_ANGLE_DEGREES); 
        return Math.sqrt(-9.8 * (Math.pow(distanceMeters, 2)) / (2 * Math.pow(Math.cos(angleRadians), 2) * (height - distanceMeters * Math.tan(angleRadians))));
    }

    /**
     * Finds angular velocity and converts to RPM by diving by 2PI and multipying by 60.
     * @param velocity
     * @return
     */
    public static double calculateReqShooterRPM(double velocity) {
        return velocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS / (2 * Math.PI) * 60; 
    }
}
