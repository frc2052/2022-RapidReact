package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;


public class OneBallAuto extends AutoBase{
    
    /**No starting position is needed, simply face the robot towards
    *target. This auto backs up to get the "off the trmac" point and shoots the preloaded ball
    *This auto is a last resort, in case of inability to use limelight or other malfunctions.
    */
    public OneBallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, null, hopper, indexer, climber);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-50),0, Rotation2d.fromDegrees(0));

        NonVisionShootCommand shootCommand = super.newNonVisionShoot1Command(7900, 7900);
        SwerveControllerCommand backUp = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, backUpPos);

        this.addCommands(shootCommand);
        this.addCommands(backUp);
    }
}