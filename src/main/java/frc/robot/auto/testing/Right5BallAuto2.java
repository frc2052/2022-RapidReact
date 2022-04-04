package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.intake.OnlyIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.subsystems.TargetingSubsystem.TargetingMode;

public class Right5BallAuto2 extends AutoBase {
    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line) Facing Towards the Hub.
     * 1. Immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * 2. Drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * 3. Drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * 4. Then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     */
    public Right5BallAuto2(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        TargetingSubsystem targeting = TargetingSubsystem.getInstance();
        
        Pose2d startPos = super.newPose2dInches(0, 0, -40);
        Pose2d ball1Pos = super.newPose2dInches(44, -38, -90);
        Pose2d behindBall2Pos = super.newPose2dInches(0, -180, 120);
        Pose2d ball2Pos = super.newPose2dInches(-10, -150, 120);
        //Pose2d shootPos = super.newPose2dInches(0, -110, -135);
        Pose2d terminalBallMidPointPos = super.newPose2dInches(-32, -287, -30);
        Pose2d terminalBallPos = super.newPose2dInches(-12, -305, -30);
        Pose2d shootPos2 = super.newPose2dInches(10, -110, 150);

        AutoTrajectoryConfig intakeBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 5, 3);
        AutoTrajectoryConfig intakeBall2TrajectoryConfig = super.createTrajectoryConfig(1.5, 1, 2, 5, 3);
        //AutoTrajectoryConfig shoot1TrajectoryConfig = super.createTrajectoryConfig(3.5, 3, 1, 8, 3);
        AutoTrajectoryConfig toTerminalTrajectoryConfig = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3.5, 3, 3, 2);
        AutoTrajectoryConfig realignTrajectoryConfig = super.createTrajectoryConfig(2.5, 1.5, 2, 5, 2);
        AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 4, 2, 3, 2);

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(intakeBall1TrajectoryConfig.withEndVelocity(1.5), startPos, ball1Pos, super.createRotationAngle(-10));
        SwerveControllerCommand driveBehindBall2 = super.createSwerveTrajectoryCommand(intakeBall2TrajectoryConfig.withStartAndEndVelocity(1.5, 1.5), super.getLastEndingPosCreated(-110), behindBall2Pos, () -> targeting.createHubTrackingSupplierWithOffset(140, 8));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(intakeBall2TrajectoryConfig, super.getLastEndingPosCreated(130), ball2Pos, super.createRotationAngle(120));
        //SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(shoot1TrajectoryConfig, super.getLastEndingPosCreated(120), shootPos, super.createHubTrackingSupplier(130));
        SwerveControllerCommand driveToTerminalMidPoint = super.createSwerveTrajectoryCommand(toTerminalTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(-30), terminalBallMidPointPos, super.createRotationAngle(-50));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(-130), terminalBallPos, super.createRotationAngle(-45));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(backToShootTrajectoryConfig, super.getLastEndingPosCreated(150), shootPos2, createHubTrackingSupplier(130));

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveAndShootToBall2 = new ParallelDeadlineGroup(driveBehindBall2, super.newIntakeArmInCommand(), new ConditionalCommand(super.newNonVisionShootAllCommand(FiringAngle.ANGLE_2, 9500, 9500, true), new WaitCommand(0.5), targeting::getIsLinedUpToShoot)); //targeting::getIsLinedUpToShoot
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(super.newAutoShoot1Command(), driveToBall2, super.newOnlyIntakeCommand());
        //ParallelDeadlineGroup driveToShoot1 = new ParallelDeadlineGroup(driveToShoot, super.newIntakeArmOutCommand());
        //ParallelDeadlineGroup aimAndShoot1 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));
        ParallelDeadlineGroup intakeTerminalBalls = new ParallelDeadlineGroup(driveToTerminalBalls, super.newAutoIntakeCommand());
        ParallelDeadlineGroup driveBackAndShoot = new ParallelDeadlineGroup(driveBackToShoot, super.newAutoTimedIntakeOnThenInCommand(2));
        ParallelDeadlineGroup aimAndShoot2 = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(super.newVisionTurnInPlaceCommand()));

        this.addCommands(super.newClimberArmsBackCommand());
        // this.addCommands(super.newInitialNonVisionShootPreloadedCommand());
        this.addCommands(intakeBall1);
        //this.andThen(() -> targeting.setTargetingMode(TargetingMode.TRACK_HUB_WITH_OFFSET));
        this.addCommands(driveAndShootToBall2);
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
        //this.addCommands(driveToShoot1);
        this.addCommands(super.newIntakeArmInCommand());
        //this.addCommands(aimAndShoot1); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalMidPoint);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newAutoIntakeCommand().withTimeout(0.75));
        this.addCommands(driveBackAndShoot);
        this.addCommands(aimAndShoot2.withTimeout(3));

        //this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}