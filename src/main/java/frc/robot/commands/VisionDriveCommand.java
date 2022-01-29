package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

public class VisionDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    //private final DoubleSupplier m_rotationSupplier;
    private VisionSubsystem m_vision;

    double visionRotation = 0;

    public VisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               //DoubleSupplier rotationSupplier,
                               VisionSubsystem vision) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        //this.m_rotationSupplier = rotationSupplier;
        this.m_vision = vision;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_vision.updateLimelight();
        //visionRotation = Math.PI;
        if(m_vision.hasValidTarget()) {
            ///visionRotation = Math.PI / 4;
            if(Math.abs(m_vision.getTx()) > 5) {
                visionRotation = -Math.copySign(Math.PI / 4, m_vision.getTx());
            } else
                visionRotation = 0;
        } else
            visionRotation = 0;
        System.out.println(visionRotation);
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        visionRotation,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
