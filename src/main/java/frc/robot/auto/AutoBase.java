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
    private TrajectoryConfig m_slowTrajectoryConfig;
    private PIDController m_slowXYController;
    private ProfiledPIDController m_slowThetaController;
    private Pose2d m_lastCreatedEndingPose;

    public AutoBase(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        m_swerveDriveKinematics = drivetrain.getKinematics();

        m_slowTrajectoryConfig = new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics);
        m_slowXYController = new PIDController(1, 0, 0);
        TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);    
        m_slowThetaController = new ProfiledPIDController(3, 0, 0, thetaControllerConstraints);
        m_slowThetaController.enableContinuousInput(-Math.PI, Math.PI);



    }

    protected SwerveControllerCommand CreateSlowDriveSlowTurnSwerveTrajectoryCommand(Pose2d startPose, Pose2d endPose) {
        m_lastCreatedEndingPose = endPose;

        SwerveControllerCommand cmd = new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                new ArrayList<Translation2d>(), //no midpoints in path (S curve)
                endPose,
                m_slowTrajectoryConfig
            ),
          m_drivetrain::getPose,
          m_swerveDriveKinematics,
          m_slowXYController,
          m_slowXYController,
          m_slowThetaController, 
          m_drivetrain::setModuleStates,
          m_drivetrain);

          return cmd;
    }

    protected SwerveControllerCommand CreateSlowDriveSlowTurnSwerveTrajectoryCommand(Pose2d startPose, Pose2d endPose, Supplier<Rotation2d> rotationSupplier) {
        m_lastCreatedEndingPose = endPose;

        SwerveControllerCommand cmd = new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                new ArrayList<Translation2d>(), //no midpoints in path (S curve)
                endPose,
                m_slowTrajectoryConfig
            ), 
          m_drivetrain::getPose, 
          m_swerveDriveKinematics,
          m_slowXYController,
          m_slowXYController,
          m_slowThetaController, 
          rotationSupplier,
          m_drivetrain::setModuleStates,
          m_drivetrain);

          return cmd;
    }

    protected Pose2d getLastEndingPosCreated() {
        return m_lastCreatedEndingPose;
    }


}
