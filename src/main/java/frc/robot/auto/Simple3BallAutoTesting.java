package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

// A version of Simple3BallAuto to be used and abused for testing and learning !

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.IntakeArmIn;
import frc.robot.commands.IntakeArmOut;
import frc.robot.commands.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class Simple3BallAutoTesting extends AutoBase {
    public Simple3BallAutoTesting(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Intake intake) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.OFF);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d start2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> aimNeg130DegreesRight = () -> { return Rotation2d.fromDegrees(-130); };

        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(-45)); //wheels should be pointing 90 degrees from straight ahead at end of path
        List<Translation2d> midpoints = List.of(new Translation2d(20, -140));
        Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(-45); };

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, start2, ball2Pos, aimNeg130DegreesRight);
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, midpoints, aimNeg45DegreesRight);
        VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);

        IntakeArmIn intakeArmIn = new IntakeArmIn(intake);
        IntakeArmOut intakeArmOut = new IntakeArmOut(intake);

        ParallelCommandGroup getBall1 = new ParallelCommandGroup(driveToBall1, intakeArmOut);

        this.addCommands(getBall1);
        this.addCommands(driveToBall2);
        this.addCommands(driveToShoot);
        this.addCommands(autoAim);


        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
