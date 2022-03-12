package frc.robot.auto.testing;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));
        Pose2d closeShootPos = super.newPose2dInches(10, 10, -160);
        Pose2d opponentBall1Pos = new Pose2d(Units.inchesToMeters(65),Units.inchesToMeters(-25), Rotation2d.fromDegrees(30)); // 30 to make it curve
        List<Translation2d> throughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(88)), new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(160)));
        Pose2d throughHangerMidpointPos = super.newPose2dInches(50, 200, -150);
        Pose2d opponentBall2Pos = super.newPose2dInches(-28, 130, -150);
        List<Translation2d> backThroughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(160)), new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(88)));
        Pose2d hideTheBallsPos = super.newPose2dInches(150, 70, -30);

        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(1, 0.5, 1, 8, 2);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        SwerveControllerCommand driveBackToShootPos = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(-160), closeShootPos, super.createHubTrackingSupplier(-160));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig.withEndVelocity(1), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-30)), opponentBall1Pos, super.createRotationAngle(-30));
        SwerveControllerCommand driveThroughHanger = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig.withStartVelocity(1), super.getLastEndingPosCreated(30), throughHangerMidpointPos, throughHangerMidpoints, super.createRotationAngle(-130));
        SwerveControllerCommand driveToOpponentBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), opponentBall2Pos, super.createRotationAngle(-150));
        SwerveControllerCommand driveToHideOpponentBalls = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(-30), hideTheBallsPos, backThroughHangerMidpoints, super.createRotationAngle(30));
        
        IntakeReverseCommand intakeRevese = new IntakeReverseCommand(intake, hopper);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup intakeOpposingBall2 = new ParallelDeadlineGroup(driveToOpponentBall2, super.newIntakeArmOutCommand());
        
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(intakeBall1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveBackToShootPos);
        this.addCommands(aimAndShoot);
        this.addCommands(intakeOpposingBall1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveThroughHanger);
        this.addCommands(intakeOpposingBall2);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveToHideOpponentBalls);
        this.addCommands(intakeRevese);
        
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}