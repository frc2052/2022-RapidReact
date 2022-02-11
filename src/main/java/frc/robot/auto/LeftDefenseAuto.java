package frc.robot.auto;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
public class LeftDefenseAuto extends AutoBase {

    /**
     * Position D Start (Far Left Parallel with Outer Tarmac Line) Facing Away from the Hub.
     * First intakes closest alliance ball, then turns and reapproaches tarmac to score 2.
     * Then drives to and intakes closest opponent cargo, and turns and fires it into the hanger.
     * (Potential for trying to get a ball that comes out of the exits with this one)
     * @param drivetrain
     * @param vision
     * @param shooter
     * @param intake
     * @param indexer
     */
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TwoWheelFlySubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(30));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(41), Units.inchesToMeters(35.5), Rotation2d.fromDegrees(30));
        //Pose2d shootPos = new Pose2d(Units.inchesToMeters(20),Units.inchesToMeters(35.5), Rotation2d.fromDegrees(160));
        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(70),Units.inchesToMeters(-10.5), Rotation2d.fromDegrees(-90));

        //AutoTrajectoryConfig fastTurnSlowDriveAutoTrajectoryConfig = super.createTrajectoryConfig(0.1, 0.1, 0.01, 10, 3, 0, 0);

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        //SwerveControllerCommand driveAndTurnToShoot = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, super.getLastEndingPosCreated(160), shootPos, super.createHubTrackingSupplier(160));
        VisionTurnInPlaceCommand turnAndAimCommand = new VisionTurnInPlaceCommand(drivetrain, vision);
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-90)), opponentBallPos, super.createRotationAngle(-90));
        TurnInPlaceCommand turnToHanger = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(135));

        IntakeArmIn intakeArmIn = new IntakeArmIn(intake);
        IntakeArmOut intakeArmOut = new IntakeArmOut(intake);

        PrepareToLaunchCargoCommand launchCargoCommand = new PrepareToLaunchCargoCommand(indexer, shooter, intake);
        
        /*
        ParallelCommandGroup intakeBall1 = new ParallelCommandGroup(driveToFirstBallPos, intakeArmOut);
        ParallelCommandGroup shootBall1 = new ParallelCommandGroup(driveAndTurnToShoot, intakeArmIn, launchCargoCommand);
        ParallelCommandGroup intakeOpposingBall1 = new ParallelCommandGroup(driveToOpponentBallPos, intakeArmOut);
        */

        this.addCommands(driveToFirstBallPos);      // intakeBall1
        this.addCommands(turnAndAimCommand);      // shootBall1
        this.addCommands(driveToOpponentBallPos);   // intakeOpposingBall1
        this.addCommands(turnToHanger);
        // this.addCommands(launchCargoCommand);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}