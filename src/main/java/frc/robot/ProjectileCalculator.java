package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class ProjectileCalculator {
    
    public double calculateUHFireAngleDeg(double distance, double height) {
        distance += Constants.Calculations.kParabolicPeakOffsetX;
        height += Constants.Calculations.kParabolicPeakOffsetY;
        return Math.atan( 2 * (-height / (distance * distance)) * (-distance));
    }

    /**
     * Calculates required velocity to send a projectile to a point with given distance and height relative to launcher, as well as the angle to fire from
     * using the equation V0 = √((-9.8d²)/(2cos²(θ)(h-d*tanθ))), which can be obtained by solving y = y0 + v0-y*t + (1/2)(g=-9.8)t² for v0, which does
     * require knowing v0-x = v0*cosθ and v0-y = v0*sinθ, and time can be solved in relation to initial velocity with x=x0+v0-x+(1/2)*a*t, leading to
     * t = x/v0-x after identifying x0 = 0 and accelaration (a) is zero because the object is in freefall meaning there are no horizontal forces.
     * 
     * I wonder if anyone will be able to understand this...
     * 
     * @param angle
     * @param distance
     * @param angle
     * @return Velocity
     */
    public double calculateReqProjectileVelocity(double angle, double distance, double height) {
        double angleRadians = Math.toRadians(angle); 
        return Math.sqrt((-Constants.Calculations.kGravitationalConstant * (distance * distance)) / (2 * Math.pow(Math.cos(angleRadians), 2) * (height - distance * Math.tan(angleRadians))));
    }
}
