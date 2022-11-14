package frc.robot.auto.tuned;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.drive.WaitOdometryResetCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class MiddleLeftHanger4BallDefenseAuto extends AutoBase {
    SwerveDriveSubsystem drivetrain;

    /**
     * Position C Start (Middle Left Parallel with Outer Tarmac line) facing inwards towards the hub.
     * First repositions slightly to get a better angle for shooting preloaded ball into the hub.
     * Then drives to and intakes the closest opponent Cargo, and shoots it in the direction of the hanger while driving to the terminal alliance Cargo.
     * Then will intake alliance cargo and wait a second in the case we choose to roll the second cargo out of the terminal to the robot.
     * Drives back to a position near it's starting location to shoot 1 or 2 cargo.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public MiddleLeftHanger4BallDefenseAuto(SwerveDriveSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, -160);
        Pose2d approachTerminalBalls = super.newPose2dInches(-152, -98, -160);
        List<Translation2d> kickBallMidpoint = List.of(super.newTranslation2dInches(-60, -54));
        Pose2d arriveAtTerminalBalls = super.newPose2dInches(-180, -109, -135);
        List<Translation2d> throughHangerMidpoint = List.of(super.newTranslation2dInches(-155, 62));
        Pose2d shootPos = super.newPose2dInches(-50, 100, -30);
        Pose2d ball4Pos = super.newPose2dInches(-20, 97, -30);

        AutoTrajectoryConfig drivingToBall2TrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 4, 2);
        AutoTrajectoryConfig intakingBothTerminalBallsTrajectoryConfig = super.createTrajectoryConfig(2.5, 1.5, 1, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 3, 2);
        AutoTrajectoryConfig intakeBall4TrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 5, 2);

        SwerveControllerCommand driveTowardsTerminalBalls = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withEndVelocity(2), startPos, approachTerminalBalls, kickBallMidpoint, super.createRotationAngle(-160));
        SwerveControllerCommand driveToArriveAtTerminalBalls = super.createSwerveTrajectoryCommand(intakingBothTerminalBallsTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-135)), arriveAtTerminalBalls, super.createRotationAngle(-135));
        SwerveControllerCommand drivebackThroughHangerToShootPos = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(90), shootPos, throughHangerMidpoint, super.createRotationAngle(-30));
        SwerveControllerCommand driveToBall4Pos = super.createSwerveTrajectoryCommand(intakeBall4TrajectoryConfig, super.getLastEndingPosCreated(-30), ball4Pos, super.createRotationAngle(-30));

        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToArriveAtTerminalBalls, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveThroughHangerToShoot = new ParallelDeadlineGroup(drivebackThroughHangerToShootPos, super.newAutoTimedIntakeOnThenInCommand(1.5));
        ParallelDeadlineGroup intakeBall4Command = new ParallelDeadlineGroup(driveToBall4Pos, super.newAutoIntakeCommand());
        ParallelDeadlineGroup aimAndShoot1 = new ParallelDeadlineGroup(super.newAutoAimAndShootAllCommandGroup(), new PerpetualCommand(super.newOnlyIntakeCommand()));
        ParallelDeadlineGroup aimingAndShooting2 = new ParallelDeadlineGroup(new PerpetualCommand(super.newAutoShootAllCommand()), new PerpetualCommand(super.newVisionTurnInPlaceCommand()), new PerpetualCommand(super.newOnlyIntakeCommand()));

        this.addCommands(new WaitOdometryResetCommand(drivetrain));
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(this.newAutoNonVisionShootAllCommand(ShootMode.SHOOT_ALL, FiringAngle.ANGLE_1, 7900, 7900));
        this.addCommands(driveTowardsTerminalBalls);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1.5));
        this.addCommands(driveThroughHangerToShoot);
        this.addCommands(super.newIntakeArmOutCommand());
        this.addCommands(aimAndShoot1);
        this.addCommands(intakeBall4Command);
        this.addCommands(aimingAndShooting2);
        this.addCommands(super.autonomousFinishedCommandGroup());

    }
}