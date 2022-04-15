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
import frc.robot.commands.drive.WaitOdometryResetCommand;
import frc.robot.commands.intake.OnlyIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class MiddleLeft4BallDefenseAuto extends AutoBase {
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
    public MiddleLeft4BallDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(-160));
        Pose2d approachTerminalBalls = new Pose2d(Units.inchesToMeters(-155), Units.inchesToMeters(-100), Rotation2d.fromDegrees(-160));
        List<Translation2d> kickBallMidpoint = List.of(new Translation2d(Units.inchesToMeters(-60), Units.inchesToMeters(-54)));
        Pose2d arriveAtTerminalBalls = new Pose2d(Units.inchesToMeters(-182),Units.inchesToMeters(-111),Rotation2d.fromDegrees(-135));

        AutoTrajectoryConfig drivingToBall2TrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 4, 2);
        AutoTrajectoryConfig intakingBothTerminalBallsTrajectoryConfig = super.createTrajectoryConfig(1.5, 1, 1, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 3, 2);

        SwerveControllerCommand driveTowardsTerminalBalls = super.createSwerveTrajectoryCommand(drivingToBall2TrajectoryConfig.withEndVelocity(3), startPos, approachTerminalBalls, kickBallMidpoint, super.createRotationAngle(-160));
        SwerveControllerCommand driveToArriveAtTerminalBalls = super.createSwerveTrajectoryCommand(intakingBothTerminalBallsTrajectoryConfig.withStartVelocity(3), super.getLastEndingPosCreated(Rotation2d.fromDegrees(-135)), arriveAtTerminalBalls, super.createRotationAngle(-135));

        OnlyIntakeCommand onlyIntakeCommand = new OnlyIntakeCommand(intake, indexer);

        //ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToArriveAtTerminalBalls, super.newIntakeArmOutCommand());
        //ParallelCommandGroup returnToShoot = new ParallelCommandGroup(drivebackThroughHangerToShootPos, super.newAutoTimedIntakeOnThenInCommand(0.5));
        ParallelDeadlineGroup aimingAndShooting1 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand())); // Perpetual Command removes the end method of a command, making it run forever.
        ParallelDeadlineGroup aimingAndShooting2 = new ParallelDeadlineGroup(new PerpetualCommand(super.newAutoShootAllCommand()), new PerpetualCommand(super.newVisionTurnInPlaceCommand()), super.newOnlyIntakeCommand());

        this.addCommands(new WaitOdometryResetCommand(drivetrain));
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(this.newInitialNonVisionShootPreloadedCommand());
        this.addCommands(driveTowardsTerminalBalls);
        this.addCommands(driveToArriveAtTerminalBalls);
        this.addCommands(aimingAndShooting1);
        this.addCommands(aimingAndShooting2);
        this.addCommands(super.autonomousFinishedCommandGroup());

    }
}