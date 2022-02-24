package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.WaitOdometryResetCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleLeft4BallTerminalDefenseAuto extends AutoBase {
    DrivetrainSubsystem drivetrain;

    /**
     * Position C Start (Middle Left Parallel with Outer Tarmac line) facing inwards towards the hub.
     * First repositions slightly to get a better angle for shooting preloaded ball into the hub.
     * Then drives to and intakes the closest opponent Cargo, and shoots it in the direction of the hanger while driving to the terminal alliance Cargo.
     * Then will intake alliance cargo and wait a second in the case we choose to roll the second cargo out of the terminal to the robot.
     * Drives back to a position near it's starting location to shoot 1 or 2 cargo.
     * 
     * WIP - Currently on MiddleLeft3BallTerminalDefense sped up.
     * 
     * @param drivetrain
     * @param vision
     * @param intake
     */
    public MiddleLeft4BallTerminalDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(-160));
        Pose2d approachTerminalBalls = new Pose2d(Units.inchesToMeters(-160), Units.inchesToMeters(-105), Rotation2d.fromDegrees(-160));
        List<Translation2d> kickBallMidpoint = List.of(new Translation2d(Units.inchesToMeters(-60), Units.inchesToMeters(-54)));
        Pose2d arriveAtTerminalBalls = new Pose2d(Units.inchesToMeters(-177),Units.inchesToMeters(-128),Rotation2d.fromDegrees(-135));
        List<Translation2d> throughHangerMidpoints = List.of(new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(-80), Units.inchesToMeters(80)));   // Midpoints for going around hanger post
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(-50), Units.inchesToMeters(80), Rotation2d.fromDegrees(30));
        //Pose2d shootPos = new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(-12), Rotation2d.fromDegrees(30));
        Pose2d ball4Pos = new Pose2d(Units.inchesToMeters(-50), Units.inchesToMeters(84), Rotation2d.fromDegrees(-30));

        AutoTrajectoryConfig drivingToBall2TrajectoryConfig = super.createTrajectoryConfig(4, 4, 1, 3, 2);
        AutoTrajectoryConfig intakingBothTerminalBallsTrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(5, 5, 1, 3, 1);
        AutoTrajectoryConfig intakeBall4TrajectoryConfig = super.createTrajectoryConfig(4, 4, 1, 5, 2);

        SwerveControllerCommand driveTowardsTerminalBalls = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withEndVelocity(3), startPos, approachTerminalBalls, kickBallMidpoint, super.createRotationAngle(-160));
        SwerveControllerCommand driveToArriveAtTerminalBalls = super.createSwerveTrajectoryCommand(intakingBothTerminalBallsTrajectoryConfig.withStartVelocity(3), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-135)), arriveAtTerminalBalls, super.createRotationAngle(-135));
        SwerveControllerCommand drivebackThroughHangerToShootPos = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(30), shootPos, throughHangerMidpoints, super.createHubTrackingSupplier(20));
        SwerveControllerCommand driveToBall4Pos = super.createSwerveTrajectoryCommand(intakeBall4TrajectoryConfig, super.getLastEndingPosCreated(-30), ball4Pos, super.createRotationAngle(-30));

        AutoShootCommand autoShootCommand = new AutoShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision);

        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToArriveAtTerminalBalls, super.newIntakeArmOutCommand());
        ParallelCommandGroup returnToShoot = new ParallelCommandGroup(drivebackThroughHangerToShootPos, super.newIntakeArmInCommand());
        ParallelDeadlineGroup waitToIntake = new ParallelDeadlineGroup(new WaitCommand(0.25), super.newIntakeArmOutCommand());
        ParallelDeadlineGroup aimingAndShooting1 = new ParallelDeadlineGroup(autoShootCommand, new PerpetualCommand(super.newVisionTurnInPlaceCommand())); // Perpetual Command removes the end method of a command, making it run forever.
        ParallelDeadlineGroup intakeBall4Command = new ParallelDeadlineGroup(driveToBall4Pos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup aimingAndShooting2 = new ParallelDeadlineGroup(autoShootCommand, new PerpetualCommand(super.newVisionTurnInPlaceCommand()));

        this.addCommands(new WaitOdometryResetCommand(drivetrain));
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(this.newNonVisionShoot1Command(7900, 7900).withTimeout(1.25));
        this.addCommands(driveTowardsTerminalBalls);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(waitToIntake);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(returnToShoot);
        this.addCommands(aimingAndShooting1);
        this.addCommands(intakeBall4Command);
        this.addCommands(aimingAndShooting2);
    }
}