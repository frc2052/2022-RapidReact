package frc.robot.auto;

// Starts at position C (Left tarmac closest to center and parallel with outermost line) and should score 2 balls (3 if we program for human player to pass ball)
// and shoot an enemy ball into the hanger.

import java.util.function.Supplier;

import javax.naming.PartialResultException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.IntakeArmOut;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class MiddleTerminalDefenseAuto extends AutoBase {
    DrivetrainSubsystem m_drivetrain;

    public MiddleTerminalDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Intake intake) {
        super(drivetrain);

        m_drivetrain = drivetrain;

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(30));
        Pose2d shootPreloadedPos = new Pose2d(Units.inchesToMeters(-24), Units.inchesToMeters(24), Rotation2d.fromDegrees(0));
        Supplier<Rotation2d> aimAtHub = () -> { return m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx())); };

        Pose2d enemyBall1Pos = new Pose2d(Units.inchesToMeters(42), Units.inchesToMeters(31.5), Rotation2d.fromDegrees(-110));
        Supplier<Rotation2d> enemyBall1Rotation = () -> { return Rotation2d.fromDegrees(-110); };

        Pose2d approachBall2 = new Pose2d(140, 93, Rotation2d.fromDegrees(30));
        Supplier<Rotation2d> rotateToFaceHanger = () -> { return Rotation2d.fromDegrees(-80); };

        Pose2d arriveAtBall2 = new Pose2d(Units.inchesToMeters(48),Units.inchesToMeters(84),Rotation2d.fromDegrees(30));
        Supplier<Rotation2d> rotateToBall2 = () -> { return Rotation2d.fromDegrees(30); };
        
        SwerveControllerCommand driveToShootPreloaded = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, shootPreloadedPos, aimAtHub);

        // ParallelCommandGroup driveAndShootPreloaded = new ParallelCommandGroup(driveToShoot1st, ShooterCommand);

        SwerveControllerCommand driveToEnemyBall1 = super.createSwerveTrajectoryCommand(super.m_fastTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-110)), enemyBall1Pos, enemyBall1Rotation);
        IntakeArmOut intakeArmOut = new IntakeArmOut(intake);

        ParallelCommandGroup getEnemyBall1CommandGroup = new ParallelCommandGroup(driveToEnemyBall1, intakeArmOut);

        SwerveControllerCommand approachBall2Command = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(30)), approachBall2, rotateToFaceHanger);

        // ParallelCommandGroup driveAndShootEnemyBall1 = new ParallelCommandGroup(approachBall2Command, ShooterCommand)

        SwerveControllerCommand arriveAtBall2Command = super.createSwerveTrajectoryCommand(super.m_fastTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(30)), arriveAtBall2, rotateToBall2);


        this.addCommands(driveToShootPreloaded); // change to driveAndShootPreloaded when shooter command ready.
        this.addCommands(getEnemyBall1CommandGroup);
        this.addCommands(approachBall2Command); // change to driveAndShootEnemyBall1 when shooter command ready
        this.addCommands(arriveAtBall2Command);
        //TODO:Shoot both loaded balls
        //TODO:Drive to rightmost ball
        //Shoot ball
        //bump right opponent ball.

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
