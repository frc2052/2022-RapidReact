package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleRightTerminal3CargoAuto extends AutoBase {

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
    public MiddleRightTerminal3CargoAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(30));
        Pose2d shootPreloadedPos = new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(-20), Rotation2d.fromDegrees(30));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(-70), Units.inchesToMeters(0), Rotation2d.fromDegrees(120));
        Pose2d approachTerminalBallPos = new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(50), Rotation2d.fromDegrees(150));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(-230), Units.inchesToMeters(32), Rotation2d.fromDegrees(150));

        SwerveControllerCommand driveToShootPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, shootPreloadedPos, createHubTrackingSupplier(30));
        SwerveControllerCommand driveToBall1Pos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(120), ball1Pos, super.createRotationAngle(120));
        SwerveControllerCommand drivetTowardsTerminalBall = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(140), approachTerminalBallPos, super.createRotationAngle(140));
        SwerveControllerCommand driveToTerminalBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(150), terminalBallPos, super.createRotationAngle(150));
        SwerveControllerCommand driveBackToShoot2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(30), shootPreloadedPos, super.createHubTrackingSupplier(30));

    //  ParallelCommandGroup intakeBall1 =  new ParallelCommandGroup(driveToBall1Pos, intakeArmOutCommand);
    //  ParallelCommandGroup approachTerminalBall = new ParallelCommandGroup(driveTowardsTerminalBall, intakeArmInCommand);
        ParallelCommandGroup intakeTerminalBall = new ParallelCommandGroup(drivetTowardsTerminalBall, super.newAutoIntakeCommand());
        ParallelCommandGroup goBackToShoot = new ParallelCommandGroup(driveBackToShoot2, super.newIntakeArmInCommand());

        this.addCommands(driveToShootPos);
        this.addCommands(super.newShoot1Command());
        this.addCommands(driveToBall1Pos);
        this.addCommands(driveToTerminalBallPos);
        this.addCommands(driveBackToShoot2);
        this.addCommands(super.newShootAllCommand());

        this.andThen(super.autonomousFinishedCommandGroup());

    }
}