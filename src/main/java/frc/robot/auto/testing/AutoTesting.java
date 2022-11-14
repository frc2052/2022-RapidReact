package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.drive.ProfiledPIDTurnInPlaceCommand;
import frc.robot.commands.drive.ProfiledPIDVisionTurnInPlaceCommand;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutoTesting extends AutoBase {

    /**
     * Class to be used and abused for testing autos
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param hopper
     * @param indexer
     * @param climber
     */
    public AutoTesting(SwerveDriveSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper, IndexerSubsystem indexer, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));

        // SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));

        // ProfiledPIDVisionTurnInPlaceCommand visionTurnInPlace = new ProfiledPIDVisionTurnInPlaceCommand(drivetrain, vision);

        // ProfiledPIDTurnInPlaceCommand turnInPlace = new ProfiledPIDTurnInPlaceCommand(drivetrain, () -> Rotation2d.fromDegrees(90));
        
        // ParallelDeadlineGroup aimAndShoot = new ParallelDeadlineGroup(super.newAutoShootAllCommand(), new PerpetualCommand(visionTurnInPlace));

        //this.addCommands(driveToFirstBallPos);
        // this.addCommands(turnInPlace);
        //this.addCommands(aimAndShoot);

        this.addCommands(new ParallelDeadlineGroup(super.newAutoShootAllCommand(100), new PerpetualCommand(super.newVisionTurnInPlaceCommand())));
    }
}
