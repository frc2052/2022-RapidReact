package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoDriveAndShootCommandV1 extends SwerveControllerCommand {
    
    private final DrivetrainSubsystem m_driveTrain;
    private final VisionSubsystem m_vision;

    private int visionRotation = 1;

    Supplier<Rotation2d> visionRotationSupplier = () -> { return Rotation2d.fromDegrees(visionRotation); };

    AutoDriveAndShootCommandV1(AutoTrajectoryConfig trajectoryConfig, Pose2d startPose, Pose2d endPose, DrivetrainSubsystem driveTrain, VisionSubsystem vision) {
        super(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                new ArrayList<Translation2d>(),
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            driveTrain::getPose,
            driveTrain.getKinematics(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(),
            visionRotationSupplier,
            driveTrain::setModuleStates,
            driveTrain
        );

        m_driveTrain = driveTrain;
        m_vision = vision;

        visionRotation = 1;
    }


}
