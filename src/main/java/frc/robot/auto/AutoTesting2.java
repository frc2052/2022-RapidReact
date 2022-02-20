package frc.robot.auto;

// A version of Simple3BallAuto to be used and abused for testing and learning !

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class AutoTesting2 extends AutoBase {

    /**
     * Position A Start (Far Right parallel with outer Tarmac line) Facing Inward Towards Hub.
     * Simple auto to score 3 alliance cargo.
     * @param drivetrain
     * @param vision
     */
    public AutoTesting2(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem grassHopper, IndexerSubsystem indexer) {
        super(drivetrain, vision);
     
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(170));
        Pose2d endPos = new Pose2d(Units.inchesToMeters(-50), 0, Rotation2d.fromDegrees(170));

        SwerveControllerCommand testDrive = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, endPos);

        this.addCommands(testDrive);
    }
}
