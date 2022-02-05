package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.AutoVisionDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class ThreeballDriveAndShoot extends AutoBase {
    public ThreeballDriveAndShoot(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);

        DrivetrainSubsystem m_drivetrain = drivetrain;
        VisionSubsystem m_vision = vision;

        // Positions and rotations
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d startPos2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
        Pose2d driveTowardsBall2 = new Pose2d(Units.inchesToMeters(18.33), Units.inchesToMeters(-80), Rotation2d.fromDegrees(-130)); // To stop at point along the way to the ball, can be figured out by making equation for the line
        Supplier<Rotation2d> aimAtHub = () -> { // Lambda that calls everything in the brackets every time the Suppier is accessed.
            m_vision.updateLimelight();
            Rotation2d rotation;
            if(m_vision.hasValidTarget()) {
                rotation = m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(179);
            }
            return rotation;
        };

        Pose2d arriveAtBall2 = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130));
        Supplier<Rotation2d> spinToBall2 = () -> {   // TODO Get pixy cam to control this in the case we're not fully aligned with the ball
            return Rotation2d.fromDegrees(-130);
        };

        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(0)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(135); };

        // Create SwerveControllerCommands
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand aimWhileDrivingToBall2 = super.createSwerveTrajectoryCommand(super.m_fastTrajectoryConfig, startPos2, driveTowardsBall2, aimAtHub);

        //ParallelCommandGroup driveAndShootToBall2 = new ParallelCommandGroup(aimWhileDrivingToBall2/*, shooterCommand*/);

        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.m_fastTrajectoryConfig, super.getLastEndingPosCreated(), arriveAtBall2, spinToBall2);
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, aimNeg45DegreesRight);

        AutoVisionDriveCommand autoAim = new AutoVisionDriveCommand(drivetrain, vision);

        // Add commands to scheduler
        this.addCommands(driveToBall1);
        this.addCommands(aimWhileDrivingToBall2); // to be changed to driveAndShootToBall2 when shooter command is ready.
        this.addCommands(driveToBall2);

        this.addCommands(driveToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(autoAim);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }


}
