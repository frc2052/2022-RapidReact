package frc.robot.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class MiddleRight5BallDefenseAuto extends AutoBase {
    public MiddleRight5BallDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TwoWheelFlySubsystem shooter, Intake intake, IndexerSubsystem indexer) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(55), Units.inchesToMeters(5), Rotation2d.fromDegrees(0));
        Pose2d approachSecondBall = new Pose2d(Units.inchesToMeters(34), Units.inchesToMeters(65), Rotation2d.fromDegrees(166));
        List<Translation2d> drivingMidpoint1 = List.of(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(75)));
        Pose2d opposingBallPos = new Pose2d(Units.inchesToMeters(-40),Units.inchesToMeters(115),Rotation2d.fromDegrees(166));
        Pose2d hangerShootPos = new Pose2d(Units.inchesToMeters(50),Units.inchesToMeters(45),Rotation2d.fromDegrees(-23));
        Pose2d approachTerminalBall = new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(-32), Rotation2d.fromDegrees(-23));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-42), Rotation2d.fromDegrees(-23));
        Pose2d driveBackToShoot = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(15), Rotation2d.fromDegrees(166));

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveAndShootToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), approachSecondBall, super.createHubTrackingSupplier(170));
        SwerveControllerCommand driveToBall2AndOpposing = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(150), opposingBallPos, drivingMidpoint1, super.createRotationAngle(150));
        SwerveControllerCommand driveAndAimBall2 = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(-23), hangerShootPos, super.createHubTrackingSupplier(-90));
        SwerveControllerCommand hangerShootToTerminalBall = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(-23), approachTerminalBall, super.createRotationAngle(-90));
        SwerveControllerCommand driveToBall3 = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(-23), terminalBallPos, super.createRotationAngle(-23));
        SwerveControllerCommand driveBackToShootFinal = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(166), driveBackToShoot, super.createHubTrackingSupplier(-166));

        IntakeArmIn intakeArmIn = new IntakeArmIn(intake);
        IntakeArmOut intakeArmOut = new IntakeArmOut(intake);

        PrepareToLaunchCargoCommand launchCargoCommand = new PrepareToLaunchCargoCommand(indexer, shooter, intake); // Adjust when ready to shoot either 1 or 2 cargo individually
/*
        ParallelCommandGroup driveAndIntakeFirstBall = new ParallelCommandGroup(driveToFirstBallPos, intakeArmOut);
        ParallelCommandGroup driveAndShootBall1 = new ParallelCommandGroup(driveAndShootToBall2, launchCargoCommand, intakeArmIn);
        ParallelCommandGroup intakeBall2AndOpposingBall1 = new ParallelCommandGroup(driveToBall2AndOpposing, intakeArmOut);
        ParallelCommandGroup driveAndShootBall2 = new ParallelCommandGroup(driveAndAimBall2, launchCargoCommand, intakeArmIn);
        ParallelCommandGroup shootEnemyBall1ToHanger = new ParallelCommandGroup(hangerShootToTerminalBall, launchCargoCommand);
        ParallelCommandGroup intakeBall3 = new ParallelCommandGroup(driveToBall3, intakeArmOut);
        ParallelCommandGroup shootFinalBall = new ParallelCommandGroup(driveBackToShootFinal, launchCargoCommand, intakeArmIn);
*/
        // Replace with parallel command groups when subsystems on robot are ready
        this.addCommands(driveToFirstBallPos);      // driveAndIntakeFirstBall
        this.addCommands(driveAndShootToBall2);     // driveAndShootBall1
        this.addCommands(driveToBall2AndOpposing);  // intakeBall2AndOpposingBall1
        this.addCommands(driveAndAimBall2);         // driveAndShootBall2
        this.addCommands(hangerShootToTerminalBall);// shootEnemyBall1ToHanger
        this.addCommands(driveToBall3);             // intakeBall3
        this.addCommands(driveBackToShootFinal);    // shootFinalBall

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}