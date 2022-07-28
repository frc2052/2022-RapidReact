package frc.robot.auto.tuned;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.climber.ClimberArmsBackCommand;
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

public class LeftDefenseAuto extends AutoBase {

    /**
     * Position D Start (Far Left Parallel with Outer Tarmac Line) Facing Away from the Hub.
     * First intakes closest alliance ball, then turns and reapproaches tarmac to score 2.
     * Then drives to and intakes closest opponent cargo, and turns and fires it into the hanger.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));
        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(65),Units.inchesToMeters(-23), Rotation2d.fromDegrees(-90));

        AutoTrajectoryConfig driveToOpponentBallBallTrajectoryConfig = super.createTrajectoryConfig(1.5, 1, 1, 5, 2);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(driveToOpponentBallBallTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-90)), opponentBallPos, super.createRotationAngle(-90));

        ClimberArmsBackCommand climberBack = new ClimberArmsBackCommand(climber);
        NonVisionShootCommand nonVisionShootAllCommand = new NonVisionShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_2, 6000, 6000);
        
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup turnToShoot = new ParallelDeadlineGroup(super.newTurnInPlaceCommand(160), super.newAutoTimedIntakeOnThenInCommand(1));
        ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup turnToHangerIntakeStayOn = new ParallelDeadlineGroup(super.newTurnInPlaceCommand(125), super.newAutoTimedIntakeOnThenInCommand(2));
        
        this.addCommands(climberBack);
        this.addCommands(intakeBall1);
        this.addCommands(turnToShoot);
        this.addCommands(aimAndShoot);
        this.addCommands(intakeOpposingBall1);
        this.addCommands(turnToHangerIntakeStayOn);
        this.addCommands(nonVisionShootAllCommand.withTimeout(1.5));
        this.addCommands(super.newTurnInPlaceCommand(-175));
          
        this.addCommands(super.autonomousFinishedCommandGroup());

        // Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        // Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));
        // Pose2d closeShootPos = super.newPose2dInches(10, 10, -160);
        // Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(65),Units.inchesToMeters(-25), Rotation2d.fromDegrees(-30));

        // AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(1, 0.5, 1, 8, 2);

        // SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        // SwerveControllerCommand driveBackToShootPos = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(-160), closeShootPos, super.createHubTrackingSupplier(-160));
        // SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-30)), opponentBallPos, super.createRotationAngle(-30));
        // TurnInPlaceCommand turnToHanger = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(30));
        // TurnInPlaceCommand turnToFirstTeleopBall = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(-175));
        
        // ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newAutoIntakeCommand());
        // ParallelDeadlineGroup backToShoot2 = new ParallelDeadlineGroup(driveBackToShootPos, super.newAutoTimedIntakeOnThenInCommand(1.5));
        // ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        // ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newAutoIntakeCommand());
        // ParallelDeadlineGroup turnToHangerIntakeStayOn = new ParallelDeadlineGroup(turnToHanger, super.newAutoIntakeCommand());
        
        // this.addCommands(super.newClimberArmsBackCommand());
        // this.addCommands(intakeBall1);
        // this.addCommands(backToShoot2);
        // this.addCommands(aimAndShoot);
        // this.addCommands(intakeOpposingBall1);
        // this.addCommands(turnToHangerIntakeStayOn);
        // this.addCommands(super.newAutoNonVisionShootAllCommand(6000, 6000));
        // this.addCommands(turnToFirstTeleopBall);
        
        // this.addCommands(super.autonomousFinishedCommandGroup());
    }
}