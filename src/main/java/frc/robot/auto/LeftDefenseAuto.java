package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class LeftDefenseAuto extends AutoBase {
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);
        vision.setLED(LEDMode.OFF);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(-106), Units.inchesToMeters(24), Rotation2d.fromDegrees(-90));
        Supplier<Rotation2d> aimNeg90 = () -> { return Rotation2d.fromDegrees(-90);};

        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(-146),Units.inchesToMeters(-21),Rotation2d.fromDegrees(-120));
        Supplier<Rotation2d> aimNeg120 = () -> { return Rotation2d.fromDegrees(-120); };

        //SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, aimNeg90);
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, firstBallPos, opponentBallPos, aimNeg120);


        //TODO: extend intake and turn on intake motors
        //TODO:Drive to left ball
        this.addCommands(driveToFirstBallPos);
        //TODO:Shoot both loaded ball
        //TODO: Drive to opponent ball
        this.addCommands(driveToOpponentBallPos);
        //TODO: Shoot opponent ball into hanger

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}