package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeballDriveAndShoot extends AutoBase {
    
    private final DrivetrainSubsystem m_drivetrain;
    private final VisionSubsystem m_vision;

    public ThreeballDriveAndShoot(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);

        m_drivetrain = drivetrain;
        m_vision = vision;

        // Positions and rotations
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d startPos2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
        Pose2d driveTowardsBall2 = new Pose2d(Units.inchesToMeters(-96), Units.inchesToMeters(-130), Rotation2d.fromDegrees(-130));
        Supplier<Rotation2d> aimAtHub = () -> { // Lambda that calls everything in the brackets every time the Suppier is accessed.
            m_vision.updateLimelight();
            Rotation2d rotation;
            rotation = m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            return rotation;
        };

        // Create SwerveControllerCommands
        SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveAndShootToBall2 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos2, driveTowardsBall2, aimAtHub);


        // Add commands to scheduler
        this.addCommands(driveToBall1);
    }
}
