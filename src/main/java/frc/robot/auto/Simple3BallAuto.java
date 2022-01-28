package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Simple3BallAuto extends AutoBase {
    public Simple3BallAuto(DrivetrainSubsystem drivetrain) {
        super(drivetrain);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(24), 0, Rotation2d.fromDegrees(0));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(60), Rotation2d.fromDegrees(90)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> point90DegreesRight = () -> { return Rotation2d.fromDegrees(90); };

        SwerveControllerCommand driveToBall1 = super.CreateSlowDriveSlowTurnSwerveTrajectoryCommand(startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.CreateSlowDriveSlowTurnSwerveTrajectoryCommand(super.getLastEndingPosCreated(), ball2Pos, point90DegreesRight);

        //TODO: extend intake and turn on intake motors
        this.addCommands(driveToBall1);
        //TOOD: shoot 2 balls
        //TODO: is there  a way to force the wheels to point 90 immediately before it starts to drive? 
        this.addCommands(driveToBall2);
        //TODO: turn towards goal        
        //TODO: shoot ball

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
