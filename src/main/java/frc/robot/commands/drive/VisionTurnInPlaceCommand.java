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

    public VisionTurnInPlaceCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        vision.setLED(LEDMode.ON);
    }

    @Override
    public void execute() {
        //double rotation = vision.getRotationToTarget();
        tx = vision.getTx();
        isLinedUp = false;

        if(vision.hasValidTarget()) {
            // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(this.tx) > 5) {
              visionRotation = -Math.copySign(Math.toRadians(this.tx * 8) , this.tx); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(this.tx) > 2) {                              // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
              visionRotation =  -Math.copySign(Math.PI /4, this.tx);
            } else {
              visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
              isLinedUp = true;
            }
          } else {
            visionRotation = 0;
          }

        if (isLinedUp) {
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_FOUND);
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
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            drivetrainSubsystem.stop();
            isLinedUp = true;
        }*/
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLED(LEDMode.OFF);
        drivetrain.stop();
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_DEFAULT);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isLinedUp;
    }
}
