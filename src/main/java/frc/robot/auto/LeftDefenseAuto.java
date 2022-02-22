package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.NonVisionShootCommand.NonVisionShootMode;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
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
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, grassHopper, indexer);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(20), Rotation2d.fromDegrees(30));
        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(65),Units.inchesToMeters(-25), Rotation2d.fromDegrees(-90));

        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, super.createRotationAngle(30));
        TurnInPlaceCommand turnToHub = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(175));
        VisionTurnInPlaceCommand turnAndAimCommand = new VisionTurnInPlaceCommand(drivetrain, vision);
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-90)), opponentBallPos, super.createRotationAngle(-90));
        TurnInPlaceCommand turnToHanger = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(135));
        TurnInPlaceCommand turnToFirstTeleopBall = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(-170));

        ClimberArmsBackCommand climberBack = new ClimberArmsBackCommand(climber);

        NonVisionShootCommand nonVisionShootAllCommand = new NonVisionShootCommand(NonVisionShootMode.SHOOT_ALL, shooter, indexer, 6000, 6000);
        
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToFirstBallPos, super.newIntakeArmOutCommand());
        //ParallelCommandGroup turnToShoot = new ParallelCommandGroup(turnToHub, super.newIntakeArmInCommand());
        ParallelDeadlineGroup intakeOpposingBall1 = new ParallelDeadlineGroup(driveToOpponentBallPos, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup turnToHangerIntakeStayOn = new ParallelDeadlineGroup(turnToHanger, super.newIntakeArmOutCommand());
        
        this.addCommands(climberBack);
        this.addCommands(intakeBall1);
        this.addCommands(turnToHub);
        this.addCommands(turnAndAimCommand);
        this.addCommands(super.newVisionTurnInPlaceCommand());
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(super.newShootAllCommand().withTimeout(4));
        this.addCommands(intakeOpposingBall1);
        this.addCommands(turnToHangerIntakeStayOn);
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(nonVisionShootAllCommand.withTimeout(3));
        this.addCommands(turnToFirstTeleopBall);

        
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}