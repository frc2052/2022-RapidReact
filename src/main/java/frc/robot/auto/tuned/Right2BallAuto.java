package frc.robot.auto.tuned;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;

import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class Right2BallAuto extends AutoBase {

     /**
     * Position A2 Start (Far Right parallel with outer Tarmac line )
     * Simple auto to score 3 alliance cargo.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public Right2BallAuto(SwerveDriveSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, -60);
        Pose2d ball1Pos = super.newPose2dInches(40, -30, 0);
        Pose2d opponentBallPos = super.newPose2dInches(33, 25, 0);

        AutoTrajectoryConfig driveToBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 4, 2);
        AutoTrajectoryConfig driveToOpponentBallTrajectoryConfig = super.createTrajectoryConfig(2, 1, 1, 3, 2);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(driveToBall1TrajectoryConfig, startPos, ball1Pos, super.createRotationAngle(-10));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(driveToOpponentBallTrajectoryConfig, super.getLastEndingPosCreated(150), opponentBallPos, super.createRotationAngle(10));

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup intakeOpponentBall = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(intakeBall1);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1));
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(super.newTurnInPlaceCommand(-150));
        this.addCommands(super.newAutoAimAndShootAllCommandGroup());
        this.addCommands(intakeOpponentBall);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1));
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(super.newTurnInPlaceCommand(-120));
        this.addCommands(super.newAutoNonVisionShootAllCommand(ShootMode.SHOOT_ALL, FiringAngle.ANGLE_2, 9500, 9500));
        this.addCommands(super.newTurnInPlaceCommand(-155));
    }
}
