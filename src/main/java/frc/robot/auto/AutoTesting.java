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
import frc.robot.commands.IntakeArmInCommand;
import frc.robot.commands.IntakeArmOutCommand;
import frc.robot.commands.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class AutoTesting extends AutoBase {
    public AutoTesting(DrivetrainSubsystem drivetrain, VisionSubsystem vision, IntakeSubsystem intake, HopperSubsystem grassHopper) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.OFF);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d testPos = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-130), Rotation2d.fromDegrees(-130));
        Supplier<Rotation2d> aimNeg130DegreesRight = () -> { return Rotation2d.fromDegrees(-130); };

        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(-45)); //wheels should be pointing 90 degrees from straight ahead at end of path
        List<Translation2d> midpoints = List.of(new Translation2d(20, -140));
        Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(-45); };

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig.withEndVelocity(2), super.getLastEndingPosCreated(-130), ball2Pos, super.createRotationAngle(-130));
        SwerveControllerCommand driveToTestPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig.withStartVelocity(2), super.getLastEndingPosCreated(-130), testPos, super.createRotationAngle(-130));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, midpoints, aimNeg45DegreesRight);
        VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);

        IntakeArmInCommand intakeArmIn = new IntakeArmInCommand(intake, grassHopper);
        IntakeArmOutCommand intakeArmOut = new IntakeArmOutCommand(intake, grassHopper);

        ParallelCommandGroup getBall1 = new ParallelCommandGroup(driveToBall1, intakeArmOut);

        this.addCommands(getBall1);
        this.addCommands(driveToBall2);
        this.addCommands(driveToTestPos);
        this.addCommands(driveToShoot);
        this.addCommands(autoAim);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
