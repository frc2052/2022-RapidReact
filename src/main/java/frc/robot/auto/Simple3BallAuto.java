package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoVisionDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class Simple3BallAuto extends AutoBase {
    public Simple3BallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain);
        vision.setLED(LEDMode.OFF);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d start2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> aimNeg130DegreesRight = () -> { return Rotation2d.fromDegrees(-130); };

        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(-45)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(-45); };

        SwerveControllerCommand driveToBall1 = super.CreateSlowDriveSlowTurnSwerveTrajectoryCommand(startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.CreateSlowDriveSlowTurnSwerveTrajectoryCommand(start2, ball2Pos, aimNeg130DegreesRight);
        SwerveControllerCommand driveToShoot = super.CreateSlowDriveSlowTurnSwerveTrajectoryCommand(super.getLastEndingPosCreated(), shootPos, aimNeg45DegreesRight);
        AutoVisionDriveCommand autoAim = new AutoVisionDriveCommand(drivetrain, vision);

        //TODO: extend intake and turn on intake motors
        this.addCommands(driveToBall1); // Drives to the closest ball to the robot
        //TOOD: shoot 2 balls
        //TODO: is there  a way to force the wheels to point 90 immediately before it starts to drive? 
        this.addCommands(driveToBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(driveToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(autoAim);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target
        vision.setLED(LEDMode.OFF);
        //TODO: shoot ball

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
