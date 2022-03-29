package frc.robot.auto.testing;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDs.LEDChannel1;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.LEDStatusMode;

public class ThreeballDriveAndShoot extends AutoBase {

    /**
    * Position A Start (Far Right Parallel with Outer Tarmac Line)
    * Varient of Simple3BallAuto originally used for testing driving while aiming at UpperHub in auto.
    * Curretnly only left for driving with horizontal offset aiming based on velocity.
    * @param drivetrain
    * @param vision
    */
    public ThreeballDriveAndShoot(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper, IndexerSubsystem indexer, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        // Positions and rotations
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
        Pose2d startPos2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
        Pose2d driveTowardsBall2 = new Pose2d(Units.inchesToMeters(18.33), Units.inchesToMeters(-80), Rotation2d.fromDegrees(-130)); // To stop at point along the way to the ball, can be figured out by making equation for the line

        Pose2d arriveAtBall2 = new Pose2d(Units.inchesToMeters(-19.5), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130));
        Supplier<Rotation2d> spinToBall2 = () -> {   // TODO Get pixy cam to control this in the case we're not fully aligned with the ball
            return Rotation2d.fromDegrees(-130);
        };

        Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(0)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(135); };

        // Create SwerveControllerCommands
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand aimWhileDrivingToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, startPos2, driveTowardsBall2, super.createHubTrackingSupplier(170));

        //ParallelCommandGroup driveAndShootToBall2 = new ParallelCommandGroup(aimWhileDrivingToBall2/*, shooterCommand*/);

        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(), arriveAtBall2, spinToBall2);
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, aimNeg45DegreesRight);

        VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);

        // Add commands to scheduler
        this.addCommands(driveToBall1);
        this.addCommands(aimWhileDrivingToBall2); // to be changed to driveAndShootToBall2 when shooter command is ready.
        this.addCommands(driveToBall2);

        this.addCommands(driveToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(autoAim);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target

        this.andThen(() -> LEDChannel1.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
