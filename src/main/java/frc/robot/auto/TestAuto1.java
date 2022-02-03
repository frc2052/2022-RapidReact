package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.TurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto1 extends SequentialCommandGroup {
  
  public TestAuto1(DrivetrainSubsystem drivetrain) {

    var swerveDriveKinematics = drivetrain.getKinematics();

    TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    var thetaController = new ProfiledPIDController(3, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

  TrajectoryConfig defaultConfig = new TrajectoryConfig(
    0.75, 0.3
  ).setKinematics(swerveDriveKinematics);
                                    
  SwerveControllerCommand forwardA = new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d()),
      new ArrayList<Translation2d>(),
      new Pose2d(2, 0, new Rotation2d()),
      defaultConfig
    ), drivetrain::getPose, 
    swerveDriveKinematics,
    new PIDController(1, 0, 0),
    new PIDController(1, 0, 0),
    thetaController, drivetrain::setModuleStates,
    drivetrain);

    SwerveControllerCommand turn = new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 0, new Rotation2d()),
        new ArrayList<Translation2d>(),
        new Pose2d(2.3, 0, Rotation2d.fromDegrees(0)),
        defaultConfig
      ), 
      drivetrain::getPose,
      swerveDriveKinematics,
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      () -> Rotation2d.fromDegrees(45),
      drivetrain::setModuleStates,
      drivetrain
    );

    SwerveControllerCommand forwardB1 = new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.3, 0, Rotation2d.fromDegrees(0)),
        new ArrayList<Translation2d>(),
        new Pose2d(2.3, 2, Rotation2d.fromDegrees(0)),
        defaultConfig
      ), 
      drivetrain::getPose,
      swerveDriveKinematics,
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      () -> Rotation2d.fromDegrees(45),
      drivetrain::setModuleStates,
      drivetrain
    );

    // addCommands(forwardA.andThen(() -> drivetrain.stop()));

    // addCommands(turn.andThen(() -> drivetrain.stop()));

    // addCommands(forwardB1.andThen(() -> drivetrain.stop()));

    addCommands(new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(90)));

  }
}
