package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class LeftTerminal3Cargo extends AutoBase {

    /**
     * Position D Start (Far Left Parallel With Outer Tarmac Edge) Facing Away From the Hub.
     * Auto for driving to terminal cargo from position D (not likely to be used)
     * (UNWRITTEN)
     * @param drivetrain
     * @param vision
     */
    /*
    1 drive to ball & out intake
    2 shoot two & in intake
    3 drive terminal midpoint & out intake
    4 drive to terminal spot
    5 drive to shooting spot and shoot
    */
    public LeftTerminal3Cargo(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
        super(drivetrain, vision, shooter, intake, grassHopper, indexer);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(30));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(41), Units.inchesToMeters(35.5), Rotation2d.fromDegrees(30));
        Pose2d terminalMidPointPos = new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(260), Rotation2d.fromDegrees(70));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(284), Rotation2d.fromDegrees(70));
        Pose2d endShootPos = new Pose2d(Units.inchesToMeters(-28), Units.inchesToMeters(130), Rotation2d.fromDegrees(210));

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        SwerveControllerCommand driveToTerminalMidPointsPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), terminalMidPointPos, super.createRotationAngle(70));
        SwerveControllerCommand driveToTerminalPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), terminalBallPos, super.createRotationAngle(70));
        SwerveControllerCommand driveToEndShootPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), endShootPos, super.createRotationAngle(210));

        IntakeArmInCommand intakeArmIn = new IntakeArmInCommand(intake, indexer, grassHopper);
        IntakeArmOutCommand intakeArmOut = new IntakeArmOutCommand(intake, indexer, grassHopper);

        ShootCommand shoot2CargoCommand = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, grassHopper, vision);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, intakeArmOut);
        ParallelDeadlineGroup terminalMidPoint = new ParallelDeadlineGroup(driveToTerminalMidPointsPos, intakeArmOut);
        ParallelDeadlineGroup intakeShootPos = new ParallelDeadlineGroup(driveToEndShootPos, intakeArmIn);

        this.addCommands(intakeBall1);
        this.addCommands(intakeArmIn);
        this.addCommands(shoot2CargoCommand);
        this.addCommands(terminalMidPoint);
        this.addCommands(driveToTerminalPos);
        this.addCommands(intakeShootPos);
        this.addCommands(shoot2CargoCommand);

    }
}
