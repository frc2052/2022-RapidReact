package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class FiveBallDreamAuto extends AutoBase {

    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line).
     * Will immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * Will then drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * Will drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * Will then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     */
    public FiveBallDreamAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.OFF);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(25), Units.inchesToMeters(58), Rotation2d.fromDegrees(0));
        Pose2d leftmostBallPos = new Pose2d(Units.inchesToMeters(48),Units.inchesToMeters(84),Rotation2d.fromDegrees(0));
        Supplier<Rotation2d> aim = () -> { return Rotation2d.fromDegrees(-130); };
        
        //SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos);
        SwerveControllerCommand driveToleftmostBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, firstBallPos, leftmostBallPos);


        //TODO: extend intake and turn on intake motors
        //TODO:Drive to middle ball
        this.addCommands(driveToFirstBallPos);
        //TODO:Shoot both loaded balls
        //TODO:Drive to rightmost ball
        //Shoot ball
        //bump right opponent ball.

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
