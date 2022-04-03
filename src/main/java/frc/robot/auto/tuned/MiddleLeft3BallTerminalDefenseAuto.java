package frc.robot.auto.tuned;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleLeft3BallTerminalDefenseAuto extends AutoBase {
    DrivetrainSubsystem drivetrain;

    /**
     * Position C Start (Middle Left Parallel with Outer Tarmac line) facing inwards towards the hub.
     * First repositions slightly to get a better angle for shooting preloaded ball into the hub.
     * Then drives to and intakes the closest opponent Cargo, and shoots it in the direction of the hanger while driving to the terminal alliance Cargo.
     * Then will intake alliance cargo and wait a second in the case we choose to roll the second cargo out of the terminal to the robot.
     * Drives back to a position near it's starting location to shoot 1 or 2 cargo.
     * TUNED
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public MiddleLeft3BallTerminalDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(-160));
        Pose2d approachTerminalBalls = new Pose2d(Units.inchesToMeters(-150), Units.inchesToMeters(-90), Rotation2d.fromDegrees(-160));
        List<Translation2d> kickBallMidpoint = List.of(new Translation2d(Units.inchesToMeters(-60), Units.inchesToMeters(-54)));
        Pose2d arriveAtTerminalBalls = new Pose2d(Units.inchesToMeters(-181),Units.inchesToMeters(-119),Rotation2d.fromDegrees(-135));
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(-12), Rotation2d.fromDegrees(30));

        AutoTrajectoryConfig drivingToBall2TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 2, 1);
        AutoTrajectoryConfig intakingBothTerminalBallsTrajectoryConfig = super.createTrajectoryConfig(1, 0.75, 1, 5, 2);
        AutoTrajectoryConfig drivingBacktTrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 3, 1);

        SwerveControllerCommand driveTowardsTerminalBalls = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withEndVelocity(1), startPos, approachTerminalBalls, kickBallMidpoint, super.createRotationAngle(-160));
        SwerveControllerCommand driveToArriveAtTerminalBalls = super.createSwerveTrajectoryCommand(intakingBothTerminalBallsTrajectoryConfig.withStartVelocity(1), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-135)), arriveAtTerminalBalls, super.createRotationAngle(-135));
        SwerveControllerCommand drivebackToShootPos = super.createSwerveTrajectoryCommand(drivingBacktTrajectoryConfig, super.getLastEndingPosCreated(30), shootPos, super.createHubTrackingSupplier(20));

        AutoShootCommand autoShootCommand = new AutoShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision, drivetrain);

        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToArriveAtTerminalBalls, super.newIntakeArmOutCommand());
        ParallelCommandGroup returnToShoot = new ParallelCommandGroup(drivebackToShootPos, super.newAutoTimedIntakeOnThenInCommand(0.5));
        //ParallelDeadlineGroup waitToIntake = new ParallelDeadlineGroup(new WaitCommand(1), super.newIntakeArmOutCommand());
        ParallelDeadlineGroup aimingAndShooting = new ParallelDeadlineGroup(autoShootCommand, new PerpetualCommand(super.newVisionTurnInPlaceCommand())); // Perpetual Command removes the end method of a command, making it run forever.

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(this.newNonVisionShoot1Command(7900, 7900).withTimeout(1));
        this.addCommands(driveTowardsTerminalBalls);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newIntakeArmOutCommand().withTimeout(1));
        this.addCommands(returnToShoot);
        this.addCommands(aimingAndShooting);
        this.addCommands(super.autonomousFinishedCommandGroup());

        // Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(30));
        // Pose2d shootPreloadedPos = new Pose2d(Units.inchesToMeters(-24), Units.inchesToMeters(24), Rotation2d.fromDegrees(0));
        // Pose2d enemyBall1Pos = new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(-31.5), Rotation2d.fromDegrees(-110));
        // Pose2d approachBall2 = new Pose2d(Units.inchesToMeters(-140), Units.inchesToMeters(-93), Rotation2d.fromDegrees(-150));
        // Pose2d arriveAtBall2 = new Pose2d(Units.inchesToMeters(-160),Units.inchesToMeters(-100),Rotation2d.fromDegrees(-170));

        // AutoTrajectoryConfig drivingToBall2TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 5, 2);

        // SwerveControllerCommand driveToShootPreloaded = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, shootPreloadedPos, super.createHubTrackingSupplier(0));
        // SwerveControllerCommand driveToEnemyBall1 = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-130)), enemyBall1Pos, super.createRotationAngle(-130));
        // SwerveControllerCommand driveTowardsBall2 = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-150)), approachBall2, super.createRotationAngle(120));
        // SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-170)), arriveAtBall2, super.createRotationAngle(-170));
        // SwerveControllerCommand drivebackToShootPos = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(30), shootPreloadedPos, super.createHubTrackingSupplier(30));


        // ShootCommand shoot1CargoCommand = new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, grassHopper, vision);
        // ShootCommand shoot2CargoCommand = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, grassHopper, vision);

        // NonVisionShootCommand nonVisionShoot1Command = new NonVisionShootCommand(NonVisionShootMode.SHOOT_SINGLE, shooter, indexer, 6000, 6000);

        // ParallelDeadlineGroup intakeEnemyBall1 = new ParallelDeadlineGroup(driveToEnemyBall1, super.newIntakeArmOutCommand());
        // ParallelDeadlineGroup driveAndShootEnemyBall1 = new ParallelDeadlineGroup(driveTowardsBall2, nonVisionShoot1Command);
        // ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, super.newIntakeArmOutCommand());
        // ParallelCommandGroup returnToShoot = new ParallelCommandGroup(drivebackToShootPos, super.newIntakeArmInCommand());

        // this.addCommands(driveToShootPreloaded); // driveAndShootPreloaded
        // this.addCommands(shoot1CargoCommand.withTimeout(1.5));
        // this.addCommands(intakeEnemyBall1);     // intakeEnemyBall1
        // this.addCommands(super.newIntakeArmInCommand());
        // this.addCommands(driveAndShootEnemyBall1);  // driveAndShootEnemyBall1
        // this.addCommands(intakeBall2);  // intakeBall2
        // this.addCommands(returnToShoot);   // returnToShoot
        // this.addCommands(shoot2CargoCommand.withTimeout(2.5));

        // this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        // this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}