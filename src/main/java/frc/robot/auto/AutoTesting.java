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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.TurnInPlaceCommand;
import frc.robot.commands.VisionTurnInPlaceCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.FeedOneCargoLaunchCommand;
import frc.robot.commands.shooter.FeedTwoCargoLaunchCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class AutoTesting extends AutoBase {
    public AutoTesting(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem grassHopper, IndexerSubsystem indexer) {
        super(drivetrain, vision);
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(-45)); //wheels should be pointing 90 degrees from straight ahead at end of path

        AutoTrajectoryConfig testingCongig = super.createTrajectoryConfig(1, 0.5, 1, 3, 1);

        TurnInPlaceCommand turnToBall1 = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(179));
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(testingCongig, startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(testingCongig, super.getLastEndingPosCreated(-130), ball2Pos, super.createRotationAngle(-130));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(testingCongig, super.getLastEndingPosCreated(), shootPos, super.createRotationAngle(-45));
        VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);

        IntakeArmOutCommand intakeArmOutCommand = new IntakeArmOutCommand(intake, grassHopper, indexer);
        IntakeArmInCommand intakeArmInCommand = new IntakeArmInCommand(intake, grassHopper, indexer);

        FeedOneCargoLaunchCommand shoot1CargoCommand = new FeedOneCargoLaunchCommand(shooter, indexer, vision);
        FeedTwoCargoLaunchCommand shoot2CargoCommand = new FeedTwoCargoLaunchCommand(shooter, indexer, vision);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, intakeArmOutCommand);
        ParallelCommandGroup returnToShoot = new ParallelCommandGroup(driveToShoot, intakeArmInCommand);

        this.addCommands(autoAim);
        this.addCommands(shoot1CargoCommand.withTimeout(2));
        this.addCommands(turnToBall1);
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_INTAKE_ON));
        this.addCommands(intakeBall1); // Drives to the closest ball to the robot
        this.addCommands(driveToBall2); // Drives and rotates to the second ball near the Tarmac
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_DEFAULT));
        this.addCommands(returnToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(autoAim);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target
        this.addCommands(shoot2CargoCommand.withTimeout(2));

        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
