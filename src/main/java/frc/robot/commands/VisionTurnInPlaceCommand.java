package frc.robot.commands;

// Command for simply rotating the robot to aim at the horizontal center of the upper hub. Similar to VisionDriveCommand.

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class VisionTurnInPlaceCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_vision;

    private double visionRotation = 0;
    private double tx;
    private boolean isLinedUp;

    public VisionTurnInPlaceCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem vision) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_vision = vision;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_vision.setLED(LEDMode.ON);
    }

    @Override
    public void execute() {
        m_vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
        //double rotation = m_vision.getRotationToTarget();
        tx = m_vision.getTx();
        isLinedUp = false;

        if(m_vision.hasValidTarget()) {
            // Logic to set the chassis rotation speed based on horizontal offset.
            if(Math.abs(this.tx) > 5) {
              visionRotation = -Math.copySign(Math.toRadians(this.tx * 9) , this.tx); // Dynamically changes rotation speed to be faster at a larger tx,
            } else if(Math.abs(this.tx) > 2) {                              // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
              visionRotation =  -Math.copySign(Math.PI /4, this.tx);
            } else {
              visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
              isLinedUp = true;
            }
          } else {
            visionRotation = Math.PI;
          }

        //if(m_vision.hasValidTarget() || visionRotation == 0) { // Logic to set the chassis rotation speed based on horizontal offset.
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    visionRotation,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );
            //visionRotation = m_vision.getRotationSpeedToTarget();
        /*} else {
            // TODO Use the gyro to get the possible general direction of the hub and spin towards that angle
            m_drivetrainSubsystem.stop();
            isLinedUp = true;
        }*/
    }

    @Override
    public void end(boolean interrupted) {
        m_vision.setLED(LEDMode.OFF);
        m_drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isLinedUp;
    }
}
