package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ShootCommand;
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

public class RightFiveBallAuto2 extends AutoBase {
    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line) Facing Towards the Hub.
     * 1. Immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * 2. Drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * 3. Drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * 4. Then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     */
    public RightFiveBallAuto2(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        
        Pose2d startPos = super.newPose2dInches(0, 0, 115);
        Pose2d ball1Pos = super.newPose2dInches(-68, 40, 90);
        Pose2d ball2Pos = super.newPose2dInches(-12, 130, -50);
        Pose2d shootPos = super.newPose2dInches(-18, 40, 45);
        Pose2d terminalBallMidPointPos = super.newPose2dInches(0, 210, 120);
        Pose2d terminalBallPos = super.newPose2dInches(-30, 210, -110);

        AutoTrajectoryConfig intakeBall1TrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 8, 3);
        AutoTrajectoryConfig intakeBall2TrajectoryConfig = super.createTrajectoryConfig(4, 4, 1, 3, 2);
        AutoTrajectoryConfig shoot1TrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 8, 3);
        AutoTrajectoryConfig toTerminalTrajectoryConfig = super.createTrajectoryConfig(5, 5, 3, 3, 2);
        AutoTrajectoryConfig realignTrajectoryConfig = super.createTrajectoryConfig(3, 3, 1, 5, 2);

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(intakeBall1TrajectoryConfig.withEndVelocity(2), startPos, ball1Pos, super.createRotationAngle(90));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(intakeBall2TrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(50), ball2Pos, super.createRotationAngle(50));
        SwerveControllerCommand driveToShoot1 = super.createSwerveTrajectoryCommand(shoot1TrajectoryConfig, super.getLastEndingPosCreated(-60), shootPos, super.createHubTrackingSupplier(-60));
        SwerveControllerCommand driveToTerminalMidPoint = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(110), terminalBallMidPointPos, super.createRotationAngle(110));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(120), terminalBallPos, super.createRotationAngle(120));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(-60), shootPos, createHubTrackingSupplier(-60));

        //ParallelDeadlineGroup aimAndShootPreloaded = new ParallelDeadlineGroup(super.newAutoShoot1Command(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup aimAndShoot1 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToTerminalBalls, super.newIntakeArmOutCommand());
        ParallelCommandGroup driveBackAndShoot2 = new ParallelCommandGroup(driveBackToShoot, super.newAutoTimedIntakeOnThenInCommand(0.5));
        ParallelDeadlineGroup aimAndShoot2 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(super.newNonVisionShootAllCommand(7900, 7900).withTimeout(1));
        this.addCommands(intakeBall1);
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveToShoot1);
        this.addCommands(aimAndShoot1); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalMidPoint);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(driveBackAndShoot2);
        this.addCommands(aimAndShoot2.withTimeout(3));

        //this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}