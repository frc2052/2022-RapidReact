package frc.robot.auto.testing;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.subsystems.DrivetrainSubsystem;

public class CISTestBasic extends SequentialCommandGroup {
    /**
     * Test auto for our dear friends CIS but basic and not cool because it doesn't use autobase 
     * because they need to learn how this works or something
     * @param drivetrain
     * @param shooter
     * @param indexer
     * @param hopper
     * @param climber
     */
    public CISTestBasic(DrivetrainSubsystem drivetrain) {

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(175));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-55), 0, Rotation2d.fromDegrees(175));

        SwerveDriveKinematics swerveDriveKinematics = drivetrain.getKinematics();

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.5, 1.5).setKinematics(swerveDriveKinematics);
        PIDController XYController = new PIDController(1, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand backUp = new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPos,
                new ArrayList<Translation2d>(), //no midpoints in path (S curve)
                backUpPos,
                trajectoryConfig
            ),
            drivetrain::getPose,
            swerveDriveKinematics,
            XYController,
            XYController,
            thetaController,
            () -> { return new Rotation2d(); },
            drivetrain::setModuleStates,
            drivetrain
        );

        this.addCommands(backUp);
    }
}
