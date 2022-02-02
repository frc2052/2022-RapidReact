package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoScoreWhileDrivingCommand extends CommandBase {
    private final AutoTrajectoryConfig m_slowTrajectoryConfig;

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final PIDController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final Supplier<Rotation2d> m_desiredRotation;

    private final DrivetrainSubsystem m_driveTrain;
    private final VisionSubsystem m_vision;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Pose2d m_lastCreatedEndingPose;
    
    AutoScoreWhileDrivingCommand(Pose2d startPose, Pose2d endPose, DrivetrainSubsystem driveTrain, VisionSubsystem vision) {
        m_driveTrain = driveTrain;
        m_vision = vision;
        m_swerveDriveKinematics = driveTrain.getKinematics();

        m_slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        m_trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            new ArrayList<Translation2d>(), //no midpoints in path (S curve)
            endPose,
            m_slowTrajectoryConfig.getTrajectoryConfig()
        );

        m_pose = m_driveTrain::getPose;
        m_kinematics = m_driveTrain.getKinematics();
        m_controller = m_slowTrajectoryConfig.getXYController();
        //m_outputModuleStates = 
        //m_desiredRotation = 
    }

    

    @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
