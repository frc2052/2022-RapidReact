package frc.robot.auto.tuned;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.drive.WaitOdometryResetCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class FastRight5BallAuto4 extends AutoBase {
    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line) Facing Towards the Hub.
     * 1. Immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * 2. Drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * 3. Drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * 4. Then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * RIGHT 5 BALL ADJUSTED TO BE MORE RELIABLE
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public FastRight5BallAuto4(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        
        Pose2d startPos = super.newPose2dInches(0, 0, 115);
        Pose2d ball1Pos = super.newPose2dInches(-33, 38, 90);
        List<Translation2d> behindBallMidpoint = List.of(super.newTranslation2dInches(-20, 160));
        Pose2d ball2Pos = super.newPose2dInches(27, 153, 30);
        Pose2d terminalBallMidPointPos = super.newPose2dInches(4, 264, 140);
        Pose2d terminalBallPos = super.newPose2dInches(-6, 301, 150);
        Pose2d shootPos2 = super.newPose2dInches(-2, 160, -60);
        // Pose2d startPos = super.newPose2dInches(0, 0, 115);
        // Pose2d ball1Pos = super.newPose2dInches(-25, 38, 90);
        // List<Translation2d> behindBallMidpoint = List.of(super.newTranslation2dInches(-20, 160));
        // Pose2d ball2Pos = super.newPose2dInches(27, 153, 30);
        // Pose2d terminalBallMidPointPos = super.newPose2dInches(4, 279, 140);
        // Pose2d terminalBallPos = super.newPose2dInches(-1, 302, 150);
        // Pose2d shootPos2 = super.newPose2dInches(0, 160, -60);

        AutoTrajectoryConfig intakeBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 5, 3);
        AutoTrajectoryConfig intakeBall2TrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 2, 5, 2);
        AutoTrajectoryConfig toTerminalTrajectoryConfig = super.createTrajectoryConfig(4, 3, 3, 5, 3);
        AutoTrajectoryConfig realignTrajectoryConfig = super.createTrajectoryConfig(3, 2, 2, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(4, 4, 2, 6, 2);

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(intakeBall1TrajectoryConfig.withEndVelocity(1), startPos, ball1Pos, super.createRotationAngle(170));
        SwerveControllerCommand driveAroundAndToBall2 = super.createSwerveTrajectoryCommand(intakeBall2TrajectoryConfig.withStartVelocity(1), super.getLastEndingPosCreated(90), ball2Pos, behindBallMidpoint, super.createRotationAngle(-45));
        SwerveControllerCommand driveToTerminalMidPoint = super.createSwerveTrajectoryCommand(toTerminalTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(140), terminalBallMidPointPos, super.createRotationAngle(130));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(150), terminalBallPos, super.createRotationAngle(135));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(-60), shootPos2, super.createRotationAngle(-60));

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newAutoIntakeCommand());
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveAroundAndToBall2, super.newAutoIntakeCommand());
        ParallelDeadlineGroup aimAndShoot1 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(1), new PerpetualCommand(super.newVisionTurnInPlaceCommand()), new PerpetualCommand(super.newOnlyIntakeCommand()));
        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToTerminalBalls, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveBackAndShoot = new ParallelDeadlineGroup(driveBackToShoot, super.newAutoTimedIntakeOnThenInCommand(2));
        ParallelDeadlineGroup aimAndShoot2 = new ParallelDeadlineGroup(super.newShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));

        this.addCommands(new WaitOdometryResetCommand(drivetrain));
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(super.newAutoNonVisionShootAllCommand(ShootMode.SHOOT_ALL, FiringAngle.ANGLE_1, 7900, 7900));
        this.addCommands(intakeBall1);
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(aimAndShoot1); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalMidPoint);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newStopDrivetrainCommand()); // Makes sure the drivetrain wasn't left in a wheel speed that could have been causing us to hit the terminal.
        this.addCommands(super.newAutoIntakeCommand().withTimeout(1.5));
        this.addCommands(driveBackAndShoot);
        this.addCommands(aimAndShoot2.withTimeout(4));
        this.addCommands(super.autonomousFinishedCommandGroup());

    }
}