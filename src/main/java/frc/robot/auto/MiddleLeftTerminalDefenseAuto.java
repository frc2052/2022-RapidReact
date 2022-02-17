package frc.robot.auto;

// Starts at position C (Left tarmac closest to center and parallel with outermost line) and should score 2 balls (3 if we program for human player to pass ball)
// and shoot an enemy ball into the hanger.

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class MiddleLeftTerminalDefenseAuto extends AutoBase {
    DrivetrainSubsystem drivetrain;

    /**
     * Position C Start (Middle Left Parallel with Outer Tarmac line) facing inwards towards the hub.
     * First repositions slightly to get a better angle for shooting preloaded ball into the hub.
     * Then drives to and intakes the closest opponent Cargo, and shoots it in the direction of the hanger while driving to the terminal alliance Cargo.
     * Then will intake alliance cargo and wait a second in the case we choose to roll the second cargo out of the terminal to the robot.
     * Drives back to a position near it's starting location to shoot 1 or 2 cargo.
     * @param drivetrain
     * @param vision
     * @param intake
     */
    public MiddleLeftTerminalDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TwoWheelFlySubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
        super(drivetrain, vision);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(30));
        Pose2d shootPreloadedPos = new Pose2d(Units.inchesToMeters(-24), Units.inchesToMeters(24), Rotation2d.fromDegrees(0));
        Pose2d enemyBall1Pos = new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(-31.5), Rotation2d.fromDegrees(-110));
        Pose2d approachBall2 = new Pose2d(Units.inchesToMeters(-140), Units.inchesToMeters(-93), Rotation2d.fromDegrees(-150));
        Pose2d arriveAtBall2 = new Pose2d(Units.inchesToMeters(-160),Units.inchesToMeters(-100),Rotation2d.fromDegrees(-170));

        AutoTrajectoryConfig drivingToBall2Config2mpsStart = super.createTrajectoryConfig(3, 2, 1, 5, 2, 2, 0);
        AutoTrajectoryConfig drivingToBall2Config2mpsEnd = super.createTrajectoryConfig(3, 2, 1, 5, 2, 0, 2);

        SwerveControllerCommand driveToShootPreloaded = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, shootPreloadedPos, super.createHubTrackingSupplier(0));
        SwerveControllerCommand driveToEnemyBall1 = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-130)), enemyBall1Pos, super.createRotationAngle(-130));
        SwerveControllerCommand approachBall2Command = super.createSwerveTrajectoryCommand(drivingToBall2Config2mpsEnd, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-150)), approachBall2, super.createRotationAngle(120));
        SwerveControllerCommand arriveAtBall2Command = super.createSwerveTrajectoryCommand(drivingToBall2Config2mpsStart, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-170)), arriveAtBall2, super.createRotationAngle(-170));
        SwerveControllerCommand drivebackToShootPos = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(30), shootPreloadedPos, super.createHubTrackingSupplier(30));

        IntakeArmOutCommand intakeArmOut = new IntakeArmOutCommand(intake, grassHopper);
        IntakeArmInCommand intakeArmIn = new IntakeArmInCommand(intake, grassHopper);

        PrepareToLaunchCargoCommand launchCargoCommand = new PrepareToLaunchCargoCommand(shooter, indexer, vision, grassHopper);

        /*
        ParallelCommandGroup intakeEnemyBall1 = new ParallelCommandGroup(driveToEnemyBall1, intakeArmOut);
        ParallelCommandGroup driveAndShootEnemyBall1 = new ParallelCommandGroup(approachBall2Command, ShooterCommand, intakeArmIn);
        ParallelCommandGroup intakeBall2 = new ParallelCommandGroup(arriveAtBall2, intakeArmOut);
        ParallelCommandGroup returnToShoot = new ParallelCommandGroup(drivebackToShootPos, intakeArmIn);
        */

        this.addCommands(driveToShootPreloaded); // driveAndShootPreloaded
//      this.addCommands(launchCargoCommand);
        this.addCommands(driveToEnemyBall1);     // intakeEnemyBall1
        this.addCommands(approachBall2Command);  // driveAndShootEnemyBall1
        this.addCommands(arriveAtBall2Command);  // intakeBall2
        this.addCommands(drivebackToShootPos);   // returnToShoot
//      this.addCommands(launchCargoCommand);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}