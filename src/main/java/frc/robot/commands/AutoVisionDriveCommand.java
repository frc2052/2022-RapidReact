package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

import java.util.function.DoubleSupplier;

public class AutoVisionDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    
    //private final DoubleSupplier m_rotationSupplier;
    private VisionSubsystem m_vision;

    double visionRotation = 0;
    double tx;
    boolean isLinedUp;

    public AutoVisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               VisionSubsystem vision) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        //this.m_rotationSupplier = rotationSupplier;
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
        m_vision.updateLimelight();
        tx = m_vision.getTx();
        isLinedUp = false;
        //visionRotation = Math.PI;
        if(m_vision.hasValidTarget()) {
            ///visionRotation = Math.PI / 4;
            if(Math.abs(tx) > 5) {
                visionRotation = -Math.copySign(Math.toRadians(m_vision.getTx() * 9) , tx);
            } else if(Math.abs(tx) > 2) {
                visionRotation = -Math.copySign(Math.PI /4, tx);
            } else{
                visionRotation = 0;
                isLinedUp = true;
            }

        } else
            visionRotation = 0;
        System.out.println(visionRotation);
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        visionRotation,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
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
