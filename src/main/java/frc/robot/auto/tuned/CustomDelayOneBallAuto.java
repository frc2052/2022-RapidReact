package frc.robot.auto.tuned;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.AutoBase;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class CustomDelayOneBallAuto extends AutoBase{
    
    /**
     * No starting position is needed, simply face the robot towards the hub.
     * target. This auto backs up to get the "off the trmac" point and shoots the preloaded ball
     * This auto is intended to be used either if we were unable to plan ahead with alliance partners
     * and get any good auto spot or as a last resort, in case of inability to use limelight or other malfunctions.
     * Custom Delay - Allows a number of seconds to be input into shuffleboard for how long until the robot should begin backing up.
     * @param drivetrain
     * @param shooter
     * @param indexer
     * @param hopper
     * @param climber
     */
    public CustomDelayOneBallAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber, DashboardControlsSubsystem dashboard) {
        super(drivetrain, null, shooter, null, hopper, indexer, climber);

        Timer timer = new Timer();

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(175));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-55),0, Rotation2d.fromDegrees(175));

        SwerveControllerCommand backUp = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, backUpPos);

        this.andThen(() -> timer.start());
        this.addCommands(super.newClimberArmsBackCommand());
        this.addCommands(super.newAutoNonVisionShootAllCommand(ShootMode.SHOOT_ALL, FiringAngle.ANGLE_1, 7900, 7900).withTimeout(3));
        this.addCommands(new WaitUntilCommand(() -> timer.hasElapsed(dashboard.getOneBallDelay())));
        this.addCommands(backUp);
        this.addCommands(super.autonomousFinishedCommandGroup());
    }
}