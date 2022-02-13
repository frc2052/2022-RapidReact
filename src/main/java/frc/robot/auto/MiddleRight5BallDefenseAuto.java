package frc.robot.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
    public MiddleRight5BallDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TwoWheelFlySubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(55), Units.inchesToMeters(5), Rotation2d.fromDegrees(0));
        Pose2d approachSecondBall = new Pose2d(Units.inchesToMeters(34), Units.inchesToMeters(65), Rotation2d.fromDegrees(166));
        List<Translation2d> alignWithWallMidpoint = List.of(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(75)));
        Pose2d opposingBallPos = new Pose2d(Units.inchesToMeters(-40),Units.inchesToMeters(115),Rotation2d.fromDegrees(166));
        Pose2d hangerShootPos = new Pose2d(Units.inchesToMeters(50),Units.inchesToMeters(45),Rotation2d.fromDegrees(-23));
        Pose2d approachTerminalBall = new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(-32), Rotation2d.fromDegrees(-23));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-42), Rotation2d.fromDegrees(-23));
        Pose2d driveBackToShoot = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(15), Rotation2d.fromDegrees(166));
        
        AutoTrajectoryConfig continousSpeedDriveEnd2mpsTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 1, 10, 0, 2);
        AutoTrajectoryConfig continousSpeedDriveStart2mpsTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 1, 10, 0, 2);
        AutoTrajectoryConfig continousSpeedDriveStartEnd2mpsTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 1, 10, 0, 2);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveAndShootToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), approachSecondBall, super.createHubTrackingSupplier(170));
        SwerveControllerCommand driveToBall2AndOpposing = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(150), opposingBallPos, alignWithWallMidpoint, super.createRotationAngle(150));
        SwerveControllerCommand driveAndAimBall2 = super.createSwerveTrajectoryCommand(continousSpeedDriveEnd2mpsTrajectoryConfig, super.getLastEndingPosCreated(-23), hangerShootPos, super.createHubTrackingSupplier(-110));
        SwerveControllerCommand hangerShootToTerminalBall = super.createSwerveTrajectoryCommand(continousSpeedDriveStartEnd2mpsTrajectoryConfig, super.getLastEndingPosCreated(-23), approachTerminalBall, super.createRotationAngle(-90));
        SwerveControllerCommand driveToBall3 = super.createSwerveTrajectoryCommand(continousSpeedDriveStart2mpsTrajectoryConfig, super.getLastEndingPosCreated(-23), terminalBallPos, super.createRotationAngle(-23));
        SwerveControllerCommand driveBackToShootFinal = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(166), driveBackToShoot, super.createHubTrackingSupplier(-166));

        IntakeArmInCommand intakeArmIn = new IntakeArmInCommand(intake, grassHopper);
        IntakeArmOutCommand intakeArmOut = new IntakeArmOutCommand(intake, grassHopper);

        PrepareToLaunchCargoCommand launchCargoCommand = new PrepareToLaunchCargoCommand(shooter, indexer, vision, intake, grassHopper); // Adjust when ready to shoot either 1 or 2 cargo individually

        // TODO probably use ParallelDeadlineGroup for continous SwerveControllerCommands to make sure the robot's not going to stop abruptly if the shooter doesn't fire in time.
/*      ParallelCommandGroup driveAndIntakeFirstBall = new ParallelCommandGroup(driveToFirstBallPos, intakeArmOut);
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