package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.PrepareToLaunchCargoCommand;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(30));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(41), Units.inchesToMeters(35.5), Rotation2d.fromDegrees(30));
        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(70),Units.inchesToMeters(-10.5), Rotation2d.fromDegrees(-90));

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        VisionTurnInPlaceCommand turnAndAimCommand = new VisionTurnInPlaceCommand(drivetrain, vision);
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-90)), opponentBallPos, super.createRotationAngle(-90));
        TurnInPlaceCommand turnToHanger = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(135));
        TurnInPlaceCommand turnToFirstTeleopBall = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(-170));

        IntakeArmInCommand intakeArmIn = new IntakeArmInCommand(intake, indexer, grassHopper);
        IntakeArmOutCommand intakeArmOut = new IntakeArmOutCommand(intake, indexer, grassHopper);

        PrepareToLaunchCargoCommand launchCargoCommand = new PrepareToLaunchCargoCommand(shooter, indexer, grassHopper, vision);
        
        /*
        ParallelCommandGroup intakeBall1 = new ParallelCommandGroup(driveToFirstBallPos, intakeArmOut);
        ParallelCommandGroup shootBall1 = new ParallelCommandGroup(driveAndTurnToShoot, intakeArmIn, launchCargoCommand);
        ParallelCommandGroup intakeOpposingBall1 = new ParallelCommandGroup(driveToOpponentBallPos, intakeArmOut);
        */

        this.addCommands(driveToFirstBallPos);      // intakeBall1
        this.addCommands(turnAndAimCommand);        // shootBall1
        this.addCommands(driveToOpponentBallPos);   // intakeOpposingBall1
        this.addCommands(turnToHanger);
        // this.addCommands(launchCargoCommand);
        this.addCommands(turnToFirstTeleopBall);

        
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}