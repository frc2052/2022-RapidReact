package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

import java.util.function.DoubleSupplier;

public class VisionDriveCommand extends DefaultDriveCommand {
    private VisionSubsystem m_vision;
    private DrivetrainSubsystem m_driveTrain;
    private double visionRotation = 0;
    private double horizontalAngle;
    private boolean isLinedUp;

    public VisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               VisionSubsystem vision,
                               DashboardControlsSubsystem dashboard) {
        super(drivetrainSubsystem,
        translationXSupplier,
        translationYSupplier,
        () -> { return 0.0; }, //this value will not be used because getTurnWillBeOverriden
        dashboard);

        this.m_vision = vision;
        this.m_driveTrain = drivetrainSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_vision.setLED(LEDMode.ON);
    }

    @Override
    protected double getTurnValue() {
        m_vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
        horizontalAngle = m_vision.getTx() + drivingHorizontalFiringOffsetAngleRadians();      // Horizontal offset from the Limelight's crosshair to target.
        isLinedUp = false;

        if(m_vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(horizontalAngle) > 5) {
                visionRotation = -Math.copySign(Math.toRadians(horizontalAngle * 9) , horizontalAngle); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(horizontalAngle) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
                visionRotation = -Math.copySign(Math.PI /4, horizontalAngle);
            } else {
                visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
                isLinedUp = true;
            }
        } else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            visionRotation = 0;
        }
        return visionRotation;
    }

    public boolean getIsLinedUp() {
        return isLinedUp;
    }

    private double drivingHorizontalFiringOffsetAngleRadians() {
        if(m_driveTrain.getLastWheelVelocity() < 0.2) {    // Just avoids doing all the math if we're not or barely moving anyway
            return 0.0;
        }
        // TODO calculate horizontal firing angle offset using driveTrain.getVelocity() using theta = tan^-1(d*(velocity of the robot)/(x velocity of the ball leaving the shooter)/sqrt(height^2+distance^2))
        double firingVelocity = 8.0; // [TEMP VALUE] TODO make this get the value calculated for firing the shooter 
        double lineToHub = Math.sqrt(Math.pow(Constants.Field.kUpperHubHeightMeters,2) + Math.pow(m_vision.getXDistanceToUpperHub(), 2));
        double radiansOffset = Math.atan(Math.toRadians(m_vision.getXDistanceToUpperHub()*m_driveTrain.getLastWheelVelocity()/firingVelocity/lineToHub));
        return Math.toDegrees(radiansOffset);
      }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_vision.setLED(LEDMode.OFF);
    }
}
