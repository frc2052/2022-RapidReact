package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;

import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class Right2BallAuto extends AutoBase {

     /**
     * Position A Start (Far Right parallel with outer Tarmac line)
     * Simple auto to score 3 alliance cargo.
     * TUNED AND WORKING
     * @param drivetrain
     * @param vision
     */
    public Right2BallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, 0);
        Pose2d ball1Pos = super.newPose2dInches(-50, -30, -30);
        Pose2d opponentBallPos = super.newPose2dInches(-50, 30, 30);

        AutoTrajectoryConfig driveToBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 4, 2);
        AutoTrajectoryConfig driveToOpponentBallTrajectoryConfig = super.createTrajectoryConfig(1, 0.5, 1, 5, 3);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(driveToBall1TrajectoryConfig, startPos, ball1Pos, super.createRotationAngle(-30));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(driveToOpponentBallTrajectoryConfig, super.getLastEndingPosCreated(150), opponentBallPos, super.createRotationAngle(30));

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup intakeOpponentBall = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(intakeBall1);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1));
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(super.newTurnInPlaceCommand(-160));
        this.addCommands(super.newAutoAimAndShootAllCommandGroup());
        this.addCommands(intakeOpponentBall);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1));
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(super.newTurnInPlaceCommand(165));
        this.addCommands(super.newNonVisionShootAllCommand(FiringAngle.ANGLE_2, 9000, 9000));
        this.addCommands(super.newTurnInPlaceCommand(-110));
        this.addCommands(super.newIntakeArmOutCommand());
    }
}
