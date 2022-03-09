package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Simple3BallAuto extends AutoBase {

     /**
     * Position A Start (Far Right parallel with outer Tarmac line)
     * Simple auto to score 3 alliance cargo.
     * TUNED AND WORKING
     * @param drivetrain
     * @param vision
     */
    public Simple3BallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(-175));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(-48), Units.inchesToMeters(2), Rotation2d.fromDegrees(-175));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(-16), Units.inchesToMeters(98), Rotation2d.fromDegrees(70)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(6), Units.inchesToMeters(50), Rotation2d.fromDegrees(45)); //wheels should be pointing 90 degrees from straight ahead at end of path

        AutoTrajectoryConfig testingCongig = super.createTrajectoryConfig(2, 1.5, 1, 3, 2);
        AutoTrajectoryConfig path2Config = super.createTrajectoryConfig(4, 3, 1, 5, 2);

        TurnInPlaceCommand turnToBall1 = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(175));
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(testingCongig, startPos, ball1Pos, super.createRotationAngle(175));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(path2Config, super.getLastEndingPosCreated(60), ball2Pos, super.createRotationAngle(60));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(path2Config, super.getLastEndingPosCreated(), shootPos, super.createHubTrackingSupplier(-45));

        ClimberArmsBackCommand climberBack = new ClimberArmsBackCommand(climber);
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup returnToShoot = new ParallelDeadlineGroup(driveToShoot, super.newAutoTimedIntakeOnThenInCommand(0.25));

        this.addCommands(climberBack);
        this.addCommands(super.newAutoAimAndShootAllCommandGroup());
        this.addCommands(turnToBall1);
        this.addCommands(intakeBall1); // Drives and intakes the closest ball to the robot
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
    //    this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_DEFAULT));
        this.addCommands(returnToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(new PerpetualCommand(super.newAutoAimAndShootAllCommandGroup()));

    //    this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
