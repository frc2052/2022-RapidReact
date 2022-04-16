package frc.robot.auto.tuned;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Left2Ball2DefenseAuto2 extends AutoBase {

    /**
     * Position D Start (Far Left Parallel with Outer Tarmac Line) Facing Away from the Hub.
     * First intakes closest alliance ball, then turns and reapproaches tarmac to score 2.
     * Then drives to and intakes closest opponent cargo, and turns and fires it into the hanger.
     * WORKING - NEEDS ADJUSTMENT
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public Left2Ball2DefenseAuto2(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, 0);
        Pose2d firstBallPos = super.newPose2dInches(48, 10, 30);
        Pose2d behindOpponentBall1Pos = super.newPose2dInches(90, 5, -135);
        Pose2d opponentBall1Pos = super.newPose2dInches(68, -32, -135);
        List<Translation2d> toOpponentBallMidpoint = List.of(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(70)));
        Pose2d opponentBall2Pos = super.newPose2dInches(-35, 115, 110);
        List<Translation2d> backThroughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(170)));
        Pose2d hideTheBallsPos = super.newPose2dInches(140, 95, 30);
        Pose2d readyForTeleopPos = super.newPose2dInches(70, -50, -135);

        AutoTrajectoryConfig driveToFirstBallTrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 1, 5, 2);
        AutoTrajectoryConfig driveBehindOpponentFirstBallPosTrajectoryConfig = super.createTrajectoryConfig(2, 2, 1, 5, 3);
        AutoTrajectoryConfig driveToOpponent1stBallTrajectoryConfig = super.createTrajectoryConfig(3, 2, 2, 5, 2);
        AutoTrajectoryConfig driveToSecondOpponentBallTrajectoryConfig = super.createTrajectoryConfig(4, 3.5, 1, 3, 1);
        AutoTrajectoryConfig driveToHideOpponentBallsTrajectoryConfig = super.createTrajectoryConfig(4, 4, 2, 3, 1);
        AutoTrajectoryConfig driveBackToBeginTeleopTrajectoryConfig = super.createTrajectoryConfig(4, 4, 2, 3, 1);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(driveToFirstBallTrajectoryConfig.withEndVelocity(2), startPos, firstBallPos, super.createRotationAngle(30));
        SwerveControllerCommand driveBehindOpponentFirstBallPos = super.createSwerveTrajectoryCommand(driveBehindOpponentFirstBallPosTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(-30), behindOpponentBall1Pos, super.createHubTrackingSupplier(175));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(driveToOpponent1stBallTrajectoryConfig, super.getLastEndingPosCreated(-30), opponentBall1Pos, super.createRotationAngle(-140));
        SwerveControllerCommand driveToOpponentBall2 = super.createSwerveTrajectoryCommand(driveToSecondOpponentBallTrajectoryConfig, super.getLastEndingPosCreated(130), opponentBall2Pos, toOpponentBallMidpoint, super.createRotationAngle(130));
        SwerveControllerCommand driveToHideOpponentBalls = super.createSwerveTrajectoryCommand(driveToHideOpponentBallsTrajectoryConfig, super.getLastEndingPosCreated(30), hideTheBallsPos, backThroughHangerMidpoints, super.createRotationAngle(30));
        SwerveControllerCommand driveBackToBeginTeleop = super.createSwerveTrajectoryCommand(driveBackToBeginTeleopTrajectoryConfig, super.getLastEndingPosCreated(-135), readyForTeleopPos, super.createRotationAngle(-135));
        
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveBehind = new ParallelDeadlineGroup(driveBehindOpponentFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand().withTimeout(3), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup intakeOpposingBall2 = new ParallelDeadlineGroup(driveToOpponentBall2, super.newAutoIntakeCommand());
        
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(intakeBall1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveBehind);
        this.addCommands(aimAndShoot);
        this.addCommands(intakeOpposingBall1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(intakeOpposingBall2);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveToHideOpponentBalls);
        this.addCommands(super.newStopDrivetrainCommand());
        this.addCommands(super.newOuttakeAllCommand().withTimeout(2));
        this.addCommands(driveBackToBeginTeleop);
        this.addCommands(super.newIntakeArmOutCommand());
        
        this.andThen(super.autonomousFinishedCommandGroup());
    }
}