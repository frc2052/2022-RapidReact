package frc.robot.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.TurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class MiddleRight2CargoDefenseAuto extends AutoBase {
    VisionSubsystem m_vision;
    DrivetrainSubsystem m_drivetrain;
    public MiddleRight2CargoDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);
        vision.setLED(LEDMode.ON);
        m_vision = vision;
        m_drivetrain = drivetrain;

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(55), Units.inchesToMeters(0), Rotation2d.fromDegrees(0));
        Pose2d approachSecondBall = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(60), Rotation2d.fromDegrees(170));
        Supplier<Rotation2d> aimAtHub1 = () -> { // Lambda that calls everything in the brackets every time the Suppier is accessed.
            m_vision.updateLimelight();
            Rotation2d rotation;
            if(m_vision.hasValidTarget()) {
                rotation = m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(170);
            }
            return rotation;
        };

        List<Translation2d> drivingMidpoint1 = List.of(new Translation2d(20, 80));
        Supplier<Rotation2d> aimAtOpposingBall = () -> { return Rotation2d.fromDegrees(160); };

        Pose2d secondBallPos = new Pose2d(Units.inchesToMeters(12),Units.inchesToMeters(92),Rotation2d.fromDegrees(110));
        Supplier<Rotation2d> aim = () -> { return Rotation2d.fromDegrees(110); };

        Pose2d opposingBallPos = new Pose2d(Units.inchesToMeters(-38),Units.inchesToMeters(105),Rotation2d.fromDegrees(135));
        //Supplier<Rotation2d> aimAtOpposingBall = () -> { return Rotation2d.fromDegrees(135); };

        Pose2d driveAway = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(24), Rotation2d.fromDegrees(0));
        Supplier<Rotation2d> aimAtHub2 = () -> { // Lambda that calls everything in the brackets every time the Suppier is accessed.
            m_vision.updateLimelight();
            Rotation2d rotation;
            if(m_vision.hasValidTarget()) {
                rotation = m_drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(-90);
            }
            return rotation;
        };

        //SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos);
        //TurnInPlaceCommand turnToShoot = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(180));
        SwerveControllerCommand driveAndShootToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), approachSecondBall, aimAtHub1);
        //SwerveControllerCommand driveToleftmostBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(110)), secondBallPos, aim);
        //SwerveControllerCommand driveToOpposingBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(135)), opposingBallPos, aimAtOpposingBall);
        SwerveControllerCommand pickUpBall2AndOpposing = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(), opposingBallPos, drivingMidpoint1, super.createRotationAngle(160));
        SwerveControllerCommand driveAwayCommand = super.createSwerveTrajectoryCommand(fastTurnTrajectoryConfig, super.getLastEndingPosCreated(), driveAway, aimAtHub2);

        //TODO: extend intake and turn on intake motors
        //Drive to middle ball
        this.addCommands(driveToFirstBallPos);
        this.addCommands(driveAndShootToBall2);
        //TODO:Shoot both loaded balls
        //Drive to rightmost ball
        //this.addCommands(driveToleftmostBallPos);
        this.addCommands(pickUpBall2AndOpposing);
        //Shoot ball
        //bump right opponent ball.
        //this.addCommands(driveToOpposingBallPos);
        this.addCommands(driveAwayCommand);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}