package frc.robot.auto.testing;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.OuttakeCommand.OuttakeMode;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class Left2Ball2DefenseAuto extends AutoBase {

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
     */
    public Left2Ball2DefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, 0); // new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = super.newPose2dInches(48, 10, 30); //new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));
        // Pose2d closeShootPos = super.newPose2dInches(10, 10, -160);
        Pose2d behindOpponentBall1Pos = super.newPose2dInches(95, 5, -135);
        Pose2d opponentBall1Pos = super.newPose2dInches(72, -35, -135); //new Pose2d(Units.inchesToMeters(65),Units.inchesToMeters(-25), Rotation2d.fromDegrees(-135)); // 30 to make it curve
        List<Translation2d> throughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(115), Units.inchesToMeters(150)));
        Pose2d throughHangerMidpointPos = super.newPose2dInches(25, 180, -150);
        Pose2d opponentBall2Pos = super.newPose2dInches(-28, 160, -160);
        List<Translation2d> backThroughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(150)));//, new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(88)));
        Pose2d hideTheBallsPos = super.newPose2dInches(180, 80, 30);
        Pose2d readyForTeleopPos = super.newPose2dInches(50, -70, -135);

        AutoTrajectoryConfig driveToFirstBallTrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 1, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(1, 0.5, 1, 8, 2);
        AutoTrajectoryConfig driveBehindOpponentFirstBallPosTrajectoryConfig = super.createTrajectoryConfig(2, 2, 1, 5, 3);
        AutoTrajectoryConfig driveToOpponent1stBallTrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 5, 2); //super.createTrajectoryConfig(3, 3, 1, 3, 1);
        AutoTrajectoryConfig driveThroughHangerTrajectoryConfig = super.createTrajectoryConfig(3.5, 2.5, 1, 3, 1); //super.createTrajectoryConfig(3, 3, 1, 3, 1);
        AutoTrajectoryConfig driveToOpponentBall2TrajectoryConfig = super.createTrajectoryConfig(3, 2, 2, 3, 1); //super.createTrajectoryConfig(3, 3, 1, 3, 1);
        AutoTrajectoryConfig driveToHideOpponentBallsTrajectoryConfig = super.createTrajectoryConfig(4, 4, 2, 3, 1); //super.createTrajectoryConfig(3, 3, 1, 3, 1);
        AutoTrajectoryConfig driveBackToBeginTeleopTrajectoryConfig = super.createTrajectoryConfig(4, 4, 2, 3, 1);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(driveToFirstBallTrajectoryConfig.withEndVelocity(2), startPos, firstBallPos, super.createRotationAngle(30));
        // SwerveControllerCommand driveBackToShootPos = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(-160), closeShootPos, super.createHubTrackingSupplier(-160));
        SwerveControllerCommand driveBehindOpponentFirstBallPos = super.createSwerveTrajectoryCommand(driveBehindOpponentFirstBallPosTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(-30), behindOpponentBall1Pos, super.createHubTrackingSupplier(175));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(driveToOpponent1stBallTrajectoryConfig, super.getLastEndingPosCreated(-30), opponentBall1Pos, super.createRotationAngle(-140));
        SwerveControllerCommand driveToThroughHangerPos = super.createSwerveTrajectoryCommand(driveThroughHangerTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(30), throughHangerMidpointPos, throughHangerMidpoints, super.createRotationAngle(-130));
        SwerveControllerCommand driveToOpponentBall2 = super.createSwerveTrajectoryCommand(driveToOpponentBall2TrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(), opponentBall2Pos, super.createRotationAngle(-130));
        SwerveControllerCommand driveToHideOpponentBalls = super.createSwerveTrajectoryCommand(driveToHideOpponentBallsTrajectoryConfig, super.getLastEndingPosCreated(-60), hideTheBallsPos, backThroughHangerMidpoints, super.createRotationAngle(30));
        SwerveControllerCommand driveBackToBeginTeleop = super.createSwerveTrajectoryCommand(driveBackToBeginTeleopTrajectoryConfig, super.getLastEndingPosCreated(-135), readyForTeleopPos, super.createRotationAngle(-135));
        
        //OuttakeCommand outtakeBalls = new OuttakeCommand(OuttakeMode.ALL_BALLS, intake, hopper, indexer);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveBehind = new ParallelDeadlineGroup(driveBehindOpponentFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand().withTimeout(3), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveThroughHanger = new ParallelDeadlineGroup(driveToThroughHangerPos, super.newAutoTimedIntakeOnThenInCommand(2));
        ParallelDeadlineGroup intakeOpposingBall2 = new ParallelDeadlineGroup(driveToOpponentBall2, super.newAutoIntakeCommand());
        
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(intakeBall1);
        this.addCommands(super.newIntakeArmInCommand());
        //this.addCommands(driveBackToShootPos);
        this.addCommands(driveBehind);
        this.addCommands(aimAndShoot);
        this.addCommands(intakeOpposingBall1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveThroughHanger);
        this.addCommands(intakeOpposingBall2);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveToHideOpponentBalls);
        this.addCommands(super.newIntakeArmOutCommand());
        this.andThen(new InstantCommand(() -> drivetrain.stop()));
        //this.addCommands(outtakeBalls.withTimeout(2));
        this.addCommands(new NonVisionShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_2, 4000, 4000, true).withTimeout(2));
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveBackToBeginTeleop);
        this.addCommands(super.newIntakeArmOutCommand());
        
        this.andThen(super.autonomousFinishedCommandGroup());
    }
}