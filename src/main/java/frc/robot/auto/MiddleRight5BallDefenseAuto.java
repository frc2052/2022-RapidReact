package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.NonVisionShootCommand.NonVisionShootMode;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

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
     */
    public MiddleRight5BallDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(55), Units.inchesToMeters(5), Rotation2d.fromDegrees(0));
        Pose2d approachSecondBall = new Pose2d(Units.inchesToMeters(34), Units.inchesToMeters(65), Rotation2d.fromDegrees(166));
        List<Translation2d> alignWithWallMidpoint = List.of(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(75)));
        Pose2d opposingBallPos = new Pose2d(Units.inchesToMeters(-40),Units.inchesToMeters(115),Rotation2d.fromDegrees(166));
        Pose2d hangerShootPos = new Pose2d(Units.inchesToMeters(50),Units.inchesToMeters(45),Rotation2d.fromDegrees(-23));
        Pose2d approachTerminalBall = new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(-32), Rotation2d.fromDegrees(-23));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-42), Rotation2d.fromDegrees(-23));
        Pose2d driveBackToShoot = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(15), Rotation2d.fromDegrees(166));
        
        AutoTrajectoryConfig pathToTerminalTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 1, 10);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveAndShootToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), approachSecondBall, super.createHubTrackingSupplier(170));
        SwerveControllerCommand driveToBall2AndOpposing = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(150), opposingBallPos, alignWithWallMidpoint, super.createRotationAngle(150));
        SwerveControllerCommand driveAndAimBall2 = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(-23), hangerShootPos, super.createHubTrackingSupplier(-110));
        SwerveControllerCommand hangerShootToTerminalBall = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withStartAndEndVelocity(2, 1), super.getLastEndingPosCreated(-23), approachTerminalBall, super.createRotationAngle(-90));
        SwerveControllerCommand driveToBall3 = super.createSwerveTrajectoryCommand(pathToTerminalTrajectoryConfig.withEndVelocity(1), super.getLastEndingPosCreated(-23), terminalBallPos, super.createRotationAngle(-23));
        SwerveControllerCommand driveBackToShootFinal = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(166), driveBackToShoot, super.createHubTrackingSupplier(-166));

        NonVisionShootCommand nonVisionShoot1CargoCommand = super.newNonVisionShoot1Command(3000, 3000);

        ParallelDeadlineGroup driveAndIntakeFirstBall = new ParallelDeadlineGroup(driveToFirstBallPos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall1 = new ParallelDeadlineGroup(driveAndShootToBall2, super.newShootAllCommand(), super.newIntakeArmInCommand());
        ParallelDeadlineGroup intakeBall2AndOpposingBall1 = new ParallelDeadlineGroup(driveToBall2AndOpposing, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall2 = new ParallelDeadlineGroup(driveAndAimBall2, super.newShoot1Command(), super.newIntakeArmInCommand());
        ParallelDeadlineGroup shootEnemyBall1ToHanger = new ParallelDeadlineGroup(hangerShootToTerminalBall, nonVisionShoot1CargoCommand);
        ParallelDeadlineGroup intakeBall3 = new ParallelDeadlineGroup(driveToBall3, super.newIntakeArmOutCommand());
        ParallelCommandGroup goBackToShootFinalBall = new ParallelCommandGroup(driveBackToShootFinal, super.newIntakeArmInCommand());

        this.addCommands(driveAndIntakeFirstBall);
        this.addCommands(driveAndShootBall1);
        this.addCommands(intakeBall2AndOpposingBall1);
        this.addCommands(driveAndShootBall2);
        this.addCommands(shootEnemyBall1ToHanger);
        this.addCommands(intakeBall3);
        this.addCommands(goBackToShootFinalBall);
        this.addCommands(super.newShootAllCommand().withTimeout(3));
        
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}