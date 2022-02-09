package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoBase  extends SequentialCommandGroup {
    private DrivetrainSubsystem m_drivetrain;
    private VisionSubsystem m_vision;
    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Pose2d m_lastCreatedEndingPose;

    // private TrajectoryConfig m_slowTrajectoryConfig;
    // private PIDController m_slowXYController;
    // private ProfiledPIDController m_slowThetaController;

    protected final AutoTrajectoryConfig slowTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnSlowDriveTrajectoryConfig;
    protected final AutoTrajectoryConfig speedDriveTrajectoryConfig;

    public AutoBase(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_swerveDriveKinematics = drivetrain.getKinematics();

        slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        fastTurnTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(3, 1.5).setKinematics(m_swerveDriveKinematics), // Speed of actions, 1st TrajectoryFactory value is max velocity and 2nd is max accelaration.
            new PIDController(1, 0, 0),  // The XY controller PID value
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 4*Math.PI)) // Turning PID COntroller. Increasing 1st value increases speed of turning, and the TrapezoidalProfile is our contraints of these values.
        );

        fastTurnSlowDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(0.25, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );

        speedDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(4.5, 3.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );
    }

    // Most basic deafult swerve command, automatically using slowTrajectoryConfig.
    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            slowTrajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose,
        List<Translation2d> midpointList
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            midpointList,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        m_lastCreatedEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                new ArrayList<Translation2d>(), //no midpoints in path (S curve)
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            m_drivetrain::getPose,
            m_swerveDriveKinematics,
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            m_drivetrain::setModuleStates,
            m_drivetrain
        );
    }

    // Swerve controller command for adding an ArrayList of midpoints.
    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        List<Translation2d> midpointList,
        Supplier<Rotation2d> rotationSupplier
    ) {
        m_lastCreatedEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpointList,
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            m_drivetrain::getPose,
            m_swerveDriveKinematics,
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            m_drivetrain::setModuleStates,
            m_drivetrain
        );
    }

    // protected SwerveControllerCommand CreateSlowDriveSlowTurnSwerveTrajectoryCommand(Pose2d startPose, Pose2d endPose) {
    //     m_lastCreatedEndingPose = endPose;

    //     SwerveControllerCommand cmd = new SwerveControllerCommand(
    //         TrajectoryGenerator.generateTrajectory(
    //             startPose,
    //             new ArrayList<Translation2d>(), //no midpoints in path (S curve)
    //             endPose,
    //             m_slowTrajectoryConfig
    //         ),
    //       m_drivetrain::getPose,
    //       m_swerveDriveKinematics,
    //       m_slowXYController,
    //       m_slowXYController,
    //       m_slowThetaController, 
    //       m_drivetrain::setModuleStates,
    //       m_drivetrain);

    //       return cmd;
    // }

    // protected SwerveControllerCommand CreateSlowDriveSlowTurnSwerveTrajectoryCommand(Pose2d startPose, Pose2d endPose, Supplier<Rotation2d> rotationSupplier) {
    //     m_lastCreatedEndingPose = endPose;

    //     SwerveControllerCommand cmd = new SwerveControllerCommand(
    //         TrajectoryGenerator.generateTrajectory(
    //             startPose,
    //             new ArrayList<Translation2d>(), //no midpoints in path (S curve)
    //             endPose,
    //             m_slowTrajectoryConfig
    //         ), 
    //       m_drivetrain::getPose, 
    //       m_swerveDriveKinematics,
    //       m_slowXYController,
    //       m_slowXYController,
    //       m_slowThetaController, 
    //       rotationSupplier,
    //       m_drivetrain::setModuleStates,
    //       m_drivetrain);

    //       return cmd;
    // }

    protected Supplier<Rotation2d> createRotationAngle(double angle) {
        return () -> { return Rotation2d.fromDegrees(angle); };
    }

    protected Supplier<Rotation2d> createHubTrackingSupplier(double noTargetAngle) {
        return () -> {
            Rotation2d rotation;
            m_vision.updateLimelight();
            if(m_vision.hasValidTarget()) {
                rotation = m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(m_vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(noTargetAngle);
            }
            return rotation;
        };
    }

    protected Pose2d getLastEndingPosCreated() {
        return m_lastCreatedEndingPose;
    }

    protected Pose2d getLastEndingPosCreated(double rotation) {
        return new Pose2d(m_lastCreatedEndingPose.getTranslation(), Rotation2d.fromDegrees(rotation));
    }

    protected Pose2d getLastEndingPosCreated(Rotation2d rotation) {
        return new Pose2d(m_lastCreatedEndingPose.getTranslation(), rotation);
    }

//    public enum StartPos {
//        PositionA,
//        PositionB,
//        PositionC,
//        PositionD,
//  }
}
