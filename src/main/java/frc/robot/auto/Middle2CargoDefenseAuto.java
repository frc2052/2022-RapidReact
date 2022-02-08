package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class Middle2CargoDefenseAuto extends AutoBase {
    public Middle2CargoDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);
        vision.setLED(LEDMode.OFF);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(13), Units.inchesToMeters(62), Rotation2d.fromDegrees(0));
        Pose2d leftmostBallPos = new Pose2d(Units.inchesToMeters(73),Units.inchesToMeters(-37),Rotation2d.fromDegrees(90));
        Supplier<Rotation2d> aim = () -> { return Rotation2d.fromDegrees(90); };
        Pose2d opposingBallPos = new Pose2d(0,Units.inchesToMeters(24),Rotation2d.fromDegrees(0));
        
        //SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveToleftmostBallPos = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, firstBallPos, leftmostBallPos);
        SwerveControllerCommand driveToOpposingBallPos = super.createSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, leftmostBallPos, opposingBallPos);

        //TODO: extend intake and turn on intake motors
        //TODO:Drive to middle ball
        this.addCommands(driveToFirstBallPos);
        //TODO:Shoot both loaded balls
        //TODO:Drive to rightmost ball
        this.addCommands(driveToleftmostBallPos);
        //Shoot ball
        //bump right opponent ball.
        this.addCommands(driveToOpposingBallPos);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
