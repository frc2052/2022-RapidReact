package frc.robot.auto;

import java.util.ArrayList;
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

public class AutoBase  extends SequentialCommandGroup {
    private DrivetrainSubsystem m_drivetrain;
    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Pose2d m_lastCreatedEndingPose;

    // private TrajectoryConfig m_slowTrajectoryConfig;
    // private PIDController m_slowXYController;
    // private ProfiledPIDController m_slowThetaController;

    protected final AutoTrajectoryConfig m_slowTrajectoryConfig;
    protected final AutoTrajectoryConfig m_fastTrajectoryConfig;

    public AutoBase(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        m_swerveDriveKinematics = drivetrain.getKinematics();

        m_slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        m_fastTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(0.25, 0, 0),
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );
    }

    protected SwerveControllerCommand ceateSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return ceateSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand ceateSwerveTrajectoryCommand(
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

    protected Pose2d getLastEndingPosCreated() {
        return m_lastCreatedEndingPose;
    }


}
