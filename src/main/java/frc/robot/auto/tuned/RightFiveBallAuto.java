package frc.robot.auto.tuned;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.VisionSubsystem;

public class RightFiveBallAuto extends AutoBase {
    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line) Facing Towards the Hub.
     * 1. Immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * 2. Drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * 3. Drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * 4. Then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     * @param climber
     */
    public RightFiveBallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        
        Pose2d startPos = super.newPose2dInches(0, 0, 115);
        Pose2d ball1Pos = super.newPose2dInches(-41, 38, 90);
        Pose2d ball2Pos = super.newPose2dInches(-26, 133, 50);
        Pose2d shootPos = super.newPose2dInches(0, 110, 45);
        Pose2d terminalBallMidPointPos = super.newPose2dInches(32, 287, 150);
        Pose2d terminalBallPos = super.newPose2dInches(12, 305, 150);
        Pose2d shootPos2 = super.newPose2dInches(-10, 110, -30);

        AutoTrajectoryConfig intakeBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 5, 3);
        AutoTrajectoryConfig intakeBall2TrajectoryConfig = super.createTrajectoryConfig(4, 3, 2, 3, 2);
        AutoTrajectoryConfig shoot1TrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 1, 8, 3);
        AutoTrajectoryConfig toTerminalTrajectoryConfig = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3.5, 3, 3, 2);
        AutoTrajectoryConfig realignTrajectoryConfig = super.createTrajectoryConfig(2.5, 1.5, 2, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 4, 2, 3, 2);

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(intakeBall1TrajectoryConfig.withEndVelocity(1), startPos, ball1Pos, super.createRotationAngle(175));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(intakeBall2TrajectoryConfig.withStartVelocity(1), super.getLastEndingPosCreated(50), ball2Pos, super.createRotationAngle(50));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(shoot1TrajectoryConfig, super.getLastEndingPosCreated(-60), shootPos, super.createHubTrackingSupplier(-50));
        SwerveControllerCommand driveToTerminalMidPoint = super.createSwerveTrajectoryCommand(toTerminalTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(150), terminalBallMidPointPos, super.createRotationAngle(130));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(150), terminalBallPos, super.createRotationAngle(135));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(-30), shootPos2, createHubTrackingSupplier(-50));

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newAutoIntakeCommand());
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveToShoot1 = new ParallelDeadlineGroup(driveToShoot, super.newAutoIntakeCommand());
        ParallelDeadlineGroup aimAndShoot1 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToTerminalBalls, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveBackAndShoot = new ParallelDeadlineGroup(driveBackToShoot, super.newAutoTimedIntakeOnThenInCommand(2));
        ParallelDeadlineGroup aimAndShoot2 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(super.newInitialNonVisionShootPreloadedCommand());
        this.addCommands(intakeBall1);
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(driveToShoot1);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(aimAndShoot1); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalMidPoint);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(0.75));
        this.addCommands(driveBackAndShoot);
        this.addCommands(aimAndShoot2.withTimeout(3));
        this.addCommands(super.autonomousFinishedCommandGroup());

    }
}