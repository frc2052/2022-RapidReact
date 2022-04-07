package frc.robot.auto.tuned;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class OneBallAuto extends AutoBase{
    
    /**
     * No starting position is needed, simply face the robot towards
     * target. This auto backs up to get the "off the trmac" point and shoots the preloaded ball
     * This auto is intended to be used either if we were unable to plan ahead with alliance partners
     * and get any good auto spot or as a last resort, in case of inability to use limelight or other malfunctions.
     * @param drivetrain
     * @param shooter
     * @param indexer
     * @param hopper
     * @param climber
     */
    public OneBallAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, null, shooter, null, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(175));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-55),0, Rotation2d.fromDegrees(175));

        SwerveControllerCommand backUp = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, backUpPos);

        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(super.newNonVisionShootAllCommand(FiringAngle.ANGLE_1, 7900, 7900).withTimeout(3));
        this.addCommands(backUp);
        this.addCommands(super.autonomousFinishedCommandGroup());

    }
}