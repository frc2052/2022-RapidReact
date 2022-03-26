package frc.robot.auto.testing;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleRight5BallDefenseAuto2 extends AutoBase {

    /**
    * Starts at position B (Right Middle parallel with double lines that go to the hub).
    * Intakes first ball straight ahead of it, and drives and shoots 2 cargo on its way to the second alliance cargo.
    * Then adjusts using a midpoint and 150 degree turn to drive along the wall and intake both alliance cargo 2 and opposing cargo 1.
    * Drives and shoots alliance cargo 2, then rotates to aim relativley in the direction of the hanger, and shoots opposing cargo 1.
    * Drives and rotates to intake alliance cargo 3 and possible an additional alliance cargo if timed correctly by human player.
    * Drives back to just outside the tarmac to fire all cargo into upper hub.
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     * @param climber
     */
    public MiddleRight5BallDefenseAuto2(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        TargetingSubsystem targeting = TargetingSubsystem.getInstance();

        Pose2d startPos = super.newPose2dInches(0, 0, 0);   // Start Pos
        Pose2d pos1 = super.newPose2dInches(55, 5, 0);      // Ball 1 position
        Pose2d pos2 = super.newPose2dInches(34, 65, 166);   // Intermediate point to drive to while shooting 2
        List<Translation2d> alignWithWallMidpoint = List.of(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(75)));
        Pose2d pos3 = super.newPose2dInches(-40, 115, 166); // Opponent ball position
        Pose2d pos4 = super.newPose2dInches(50, 45, -23);   // Position to switch to aim towards the hanger
        Pose2d pos5 = super.newPose2dInches(160, -32, -23); // Line up with the terminal midpoint pos
        Pose2d pos6 = super.newPose2dInches(190, -42, -23); // Terminal ball pos
        Pose2d pos7 = super.newPose2dInches(50, 15, 166);   // Ending shooting pos
        
        AutoTrajectoryConfig trajectoryConfig1 = super.createTrajectoryConfig(3.5, 3, 3, 3, 2); // Drive to first ball trajectory config
        AutoTrajectoryConfig trajectoryConfig2 = super.createTrajectoryConfig(2, 2, 1, 3, 2);   // Drive and shoot 2 trajectory config
        AutoTrajectoryConfig trajectoryConfig3 = super.createTrajectoryConfig(4, 3, 1, 3, 1);   // Drive to alliance and opponent balls trajectory config
        AutoTrajectoryConfig trajectoryConfig4 = super.createTrajectoryConfig(4, 3, 1, 1, 10);  // Path to Terminal trajectory config
        AutoTrajectoryConfig trajectoryConfig5 = super.createTrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 4, 3, 3, 2);

        SwerveControllerCommand driveCommand1 = super.createSwerveTrajectoryCommand(trajectoryConfig1, startPos, pos1);  // Drive to first ball pos
        SwerveControllerCommand driveCommand2 = super.createSwerveTrajectoryCommand(trajectoryConfig2, super.getLastEndingPosCreated(170), pos2, targeting.createHubTrackingSupplierWithOffset(170, 10)); // Drive and shoot 2
        SwerveControllerCommand driveCommand3 = super.createSwerveTrajectoryCommand(trajectoryConfig3, super.getLastEndingPosCreated(150), pos3, alignWithWallMidpoint, super.createRotationAngle(150)); // Drive to alliance ball 2 and opposing ball 1
        SwerveControllerCommand driveCommand4 = super.createSwerveTrajectoryCommand(trajectoryConfig4.withEndVelocity(2), super.getLastEndingPosCreated(-23), pos4, super.createHubTrackingSupplier(-110)); // Drive and aim ball 2
        SwerveControllerCommand driveCommand5 = super.createSwerveTrajectoryCommand(trajectoryConfig4.withStartAndEndVelocity(2, 1), super.getLastEndingPosCreated(-23), pos5, super.createRotationAngle(-90)); // Drive to terminal and aim at hanger
        SwerveControllerCommand driveCommand6 = super.createSwerveTrajectoryCommand(trajectoryConfig4.withEndVelocity(1), super.getLastEndingPosCreated(-23), pos6, super.createRotationAngle(-23)); // Drive to terminal ball pos
        SwerveControllerCommand driveCommand7 = super.createSwerveTrajectoryCommand(trajectoryConfig5, super.getLastEndingPosCreated(166), pos7, super.createHubTrackingSupplier(-166)); // Drive back to shoot final ball

        ShootCommand shootAll1 = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision, targeting::getIsLinedUpToShoot);
        ShootCommand shootAll2 = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision, targeting::getIsLinedUpToShoot);

        ParallelDeadlineGroup driveAndIntakeFirstBall       = new ParallelDeadlineGroup(driveCommand1, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall1            = new ParallelDeadlineGroup(driveCommand2, shootAll1, super.newAutoTimedIntakeOnThenInCommand(0.5));
        ParallelDeadlineGroup intakeBall2AndOpposingBall1   = new ParallelDeadlineGroup(driveCommand3, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup driveAndShootBall2            = new ParallelDeadlineGroup(driveCommand4, shootAll2, super.newIntakeArmInCommand());
        ParallelDeadlineGroup shootEnemyBall1ToHanger       = new ParallelDeadlineGroup(driveCommand5, super.newNonVisionShoot1Command(4000, 4000));
        ParallelDeadlineGroup intakeBall3                   = new ParallelDeadlineGroup(driveCommand6, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup goBackToShootFinalBall        = new ParallelDeadlineGroup(driveCommand7, super.newAutoTimedIntakeOnThenInCommand(0.5));

        this.addCommands(driveAndIntakeFirstBall);
        this.addCommands(driveAndShootBall1);
        this.addCommands(intakeBall2AndOpposingBall1);
        this.addCommands(driveAndShootBall2);
        this.addCommands(shootEnemyBall1ToHanger);
        this.addCommands(intakeBall3);
        this.addCommands(goBackToShootFinalBall);
        this.addCommands(new PerpetualCommand(super.newAutoAimAndShootAllCommandGroup()).withTimeout(5));
        
        //this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}