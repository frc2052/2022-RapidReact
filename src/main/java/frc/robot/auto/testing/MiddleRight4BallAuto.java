package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class MiddleRight4BallAuto extends AutoBase {

    /**
     * To Start at Position B (Middle Right Parallel with the 2 Lines Going to the Hub)
     * First repositions and fires preloaded cargo, then drives and intakes the nearest allicance cargo
     * and the Terminal cargo, then drives back to shoot both (Unlikely to be used)
     * 
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param hopper
     */
    public MiddleRight4BallAuto(SwerveDriveSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = super.newPose2dInches(0, 0, -30);
        Pose2d ball1Pos = super.newPose2dInches(70, 0, 120);
        Pose2d approachTerminalBallPos = super.newPose2dInches(200, -50, -30);
        Pose2d terminalBallPos = super.newPose2dInches(230, -32, -30);
        Pose2d shootPos = super.newPose2dInches(100, -32, 140);

        AutoTrajectoryConfig driveToBall1TrajectoryConfig = super.createTrajectoryConfig(3, 2, 1, 3, 2);
        AutoTrajectoryConfig driveTowardsTerminalBallTrajectoryConfig = super.createTrajectoryConfig(4, 3, 3, 3, 2);
        AutoTrajectoryConfig driveToTerminalBallPosTrajectoryConfig = super.createTrajectoryConfig(2, 1.5, 2, 5, 2);
        AutoTrajectoryConfig driveBackToShootTrajectoryConfig = super.createTrajectoryConfig(4, 3, 2, 3, 2);

        SwerveControllerCommand driveToBall1Pos = super.createSwerveTrajectoryCommand(driveToBall1TrajectoryConfig, startPos, ball1Pos, super.createRotationAngle(-30));
        SwerveControllerCommand driveTowardsTerminalBalls = super.createSwerveTrajectoryCommand(driveTowardsTerminalBallTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(140), approachTerminalBallPos, super.createRotationAngle(-30));
        SwerveControllerCommand driveToTerminalBallPos = super.createSwerveTrajectoryCommand(driveToTerminalBallPosTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(150), terminalBallPos, super.createRotationAngle(-30));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(driveBackToShootTrajectoryConfig, super.getLastEndingPosCreated(30), shootPos, super.createRotationAngle(140));

        ParallelCommandGroup intakeTerminalBalls = new ParallelCommandGroup(driveToTerminalBallPos, super.newAutoIntakeCommand());

        this.addCommands(driveToBall1Pos);
        this.addCommands(super.newTurnInPlaceCommand(175));
        this.addCommands(super.newAutoAimAndShootAllCommandGroup());
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveTowardsTerminalBalls);
        this.addCommands(intakeTerminalBalls);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(driveBackToShoot);
        this.addCommands(super.newShootAllCommand());

        this.andThen(super.autonomousFinishedCommandGroup());

    }
}
