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
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleRight5BallDefenseAuto extends AutoBase {

    /**
    * Starts at position B (Right Middle parallel with double lines that go to the hub).
    * Intakes first ball straight ahead of it, and drives and shoots 2 cargo on its way to the second alliance cargo.
    * Then adjusts using a midpoint and 150 degree turn to drive along the wall and intake both alliance cargo 2 and opposing cargo 1.
    * Drives and shoots alliance cargo 2, then rotates to aim relativley in the direction of the hanger, and shoots opposing cargo 1.
    * Drives and rotates to intake alliance cargo 3 and possible an additional alliance cargo if timed correctly by human player.
    * Drives back to just outside the tarmac to fire all cargo into upper hub.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param climber
     */
    public MiddleRight5BallDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        TargetingSubsystem targeting = TargetingSubsystem.getInstance();

        Pose2d startPos = super.newPose2dInches(0, 0, 0);
        Pose2d firstBallPos = super.newPose2dInches(55, 5, 0);
        Pose2d approachSecondBall = super.newPose2dInches(34, 65, 166);
        List<Translation2d> alignWithWallMidpoint = List.of(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(75)));
        Pose2d opposingBallPos = super.newPose2dInches(-40, 115, 166);
        Pose2d hangerShootPos = super.newPose2dInches(50, 45, -23);
        Pose2d approachTerminalBall = super.newPose2dInches(160, -32, -23);
        Pose2d terminalBallPos = super.newPose2dInches(190, -42, -23);
        Pose2d driveBackToShoot = super.newPose2dInches(50, 15, 166);
        
        AutoTrajectoryConfig driveToFirstBallTrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 3, 3, 2);
        AutoTrajectoryConfig driveAndShoot2TrajectoryConfig = super.createTrajectoryConfig(2, 2, 1, 3, 2);
        AutoTrajectoryConfig driveToBall2AndOpposingTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 3, 1);   // Drive to alliance and opponent balls trajectory config
        AutoTrajectoryConfig pathToTerminalTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 1, 10);
        AutoTrajectoryConfig driveBackToShootTracjectoryConfig = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 4, 3, 3, 2);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(driveToFirstBallTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveAndShoot2 = super.createSwerveTrajectoryCommand(driveAndShoot2TrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), approachSecondBall, () -> targeting.createHubTrackingSupplierWithOffset(170, 10));
        SwerveControllerCommand driveToBall2AndOpposing = super.createSwerveTrajectoryCommand(driveToBall2AndOpposingTrajectoryConfig, super.getLastEndingPosCreated(150), opposingBallPos, alignWithWallMidpoint, super.createRotationAngle(150));
        SwerveControllerCommand driveAndAimBall2 = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(-23), hangerShootPos, super.createHubTrackingSupplier(-110));
        SwerveControllerCommand hangerShootToTerminalBall = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withStartAndEndVelocity(2, 1), super.getLastEndingPosCreated(-23), approachTerminalBall, super.createRotationAngle(-90));
        SwerveControllerCommand driveToBall3 = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withEndVelocity(1), super.getLastEndingPosCreated(-23), terminalBallPos, super.createRotationAngle(-23));
        SwerveControllerCommand driveBackToShootFinal = super.createSwerveTrajectoryCommand(driveBackToShootTracjectoryConfig, super.getLastEndingPosCreated(166), driveBackToShoot, super.createHubTrackingSupplier(-166));

        ParallelDeadlineGroup driveAndIntakeFirstBall = new ParallelDeadlineGroup(driveToFirstBallPos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall1 = new ParallelDeadlineGroup(driveAndShoot2, super.newShootAllCommand(), super.newAutoTimedIntakeOnThenInCommand(0.5));
        ParallelDeadlineGroup intakeBall2AndOpposingBall1 = new ParallelDeadlineGroup(driveToBall2AndOpposing, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall2 = new ParallelDeadlineGroup(driveAndAimBall2, super.newShoot1Command(), super.newIntakeArmInCommand());
        ParallelDeadlineGroup shootEnemyBall1ToHanger = new ParallelDeadlineGroup(hangerShootToTerminalBall, super.newNonVisionShoot1Command(4000, 4000));
        ParallelDeadlineGroup intakeBall3 = new ParallelDeadlineGroup(driveToBall3, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup goBackToShootFinalBall = new ParallelDeadlineGroup(driveBackToShootFinal, super.newAutoTimedIntakeOnThenInCommand(0.5));

        this.addCommands(driveAndIntakeFirstBall);
        this.addCommands(driveAndShootBall1);
        this.addCommands(intakeBall2AndOpposingBall1);
        this.addCommands(driveAndShootBall2);
        this.addCommands(shootEnemyBall1ToHanger);
        this.addCommands(intakeBall3);
        this.addCommands(goBackToShootFinalBall);
        this.addCommands(new PerpetualCommand(super.newAutoAimAndShootAllCommandGroup()).withTimeout(5));
        
        //this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}