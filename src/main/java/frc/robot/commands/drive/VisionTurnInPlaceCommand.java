package frc.robot.commands.drive;

// Command for simply rotating the robot to aim at the horizontal center of the upper hub. Similar to VisionDriveCommand.

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class VisionTurnInPlaceCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;

    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    /**
     * Command to turn in place to aim the robot at the hub.
     * @param drivetrain
     * @param vision
     */
    public VisionTurnInPlaceCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        vision.setLEDMode(LEDMode.ON);
    }

    // Same relatively arbitrary aiming logic based on limelight tx found in VisionDriveCommand
    @Override
    public void execute() {
        //double rotation = vision.getRotationToTarget();
        tx = vision.getTx();
        isLinedUp = false;

        if(vision.getHasValidTarget()) {
            // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(tx) > 5) {
              visionRotation = -Math.copySign(Math.toRadians(tx * 8) , tx); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(tx) > 2) {                              // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
              visionRotation =  -Math.copySign(Math.PI /4, tx);
            } else {
              visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
              isLinedUp = true;
            }
          } else {
            visionRotation = 0;
          }

            if (isLinedUp) {
                LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_LINED_UP);
            } else {
                LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGETING);
            }

        //if(vision.hasValidTarget() || visionRotation == 0) { // Logic to set the chassis rotation speed based on horizontal offset.
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    visionRotation,
                    drivetrain.getGyroscopeRotation()
                )
            );
            //visionRotation = vision.getRotationSpeedToTarget();
        /*} else {
            drivetrainSubsystem.stop();
            isLinedUp = true;
        }*/
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLEDMode(LEDMode.OFF);
        drivetrain.stop();
        LEDSubsystem.getInstance().setLEDStatusMode(null);
    }

    @Override
    public boolean isFinished() {
        return isLinedUp;
    }
}
